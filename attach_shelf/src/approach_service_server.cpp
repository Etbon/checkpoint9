#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>
#include <chrono>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

using GoToLoading = attach_shelf::srv::GoToLoading;

class ApproachServer : public rclcpp::Node {
  public:
    ApproachServer()
        : Node("approach_service_server_node") {

        // Initialize subscriber 
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::SensorDataQoS(),
            std::bind(&ApproachServer::laser_callback, this, std::placeholders::_1)
        );
    
        // Initialize server
        server_ = this->create_service<GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Initialize static TF
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this); 

        // Initialize buffer 
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        
        // Initialize listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize velocity publisher
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

        // Initialize elevator publisher
        elevator_up_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    }

  private:
    // Define member variables 
    rclcpp::Service<GoToLoading>::SharedPtr server_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_; // Latest scan 
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // TF listenig 
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Motion I/O
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;


    //----------------
    // Laser callback
    //----------------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laser_scan_ = msg;  // save current laser scan
    }

    //------------------
    // Find shelf 
    //------------------
    std::vector<int> find_reflective_indices(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

        std::vector<int> indices;  // Create vector 

        // Safety guard - laser data 
        if (!scan){                
            RCLCPP_WARN(this->get_logger(),"No laser data found. Skipping detection.");
            return indices;
        }   

        const size_t N = scan->ranges.size();

        // Safty gards -  match laser size   intensities
        if (scan->intensities.empty() || scan->intensities.size() != N) {
            RCLCPP_WARN(this->get_logger(),"Mismatched intensities: ranges=%zu intensities=%zu",
                N, scan->intensities.size()
            );
            
            return indices;
        }
        
        // Make pre set space for vector
        indices.reserve(N / 10);

        // Pass through laser data 
        for (int i = 0; i < static_cast<int>(N); ++i) {
            float r = scan->ranges[i];
            float I = scan->intensities[i];

            // Filter NaN & Inf    
            if (!std::isfinite(r)){
                continue;
            } 
            // Intencety 
            if (I >= 8000.0f) {
                indices.push_back(i);  // save index 
            }
        }

        return indices;    
    }

    //------------------
    // Find shelf legs 
    //------------------
    std::vector<std::vector<int>> find_shelf_leg_cluster(const std::vector<int> &laser_index) {
        std::vector<std::vector<int>> clusters; // all groups 
        std::vector<int> current_cluster;  // one group

        if (laser_index.empty()) return clusters; // safty gaurd

        current_cluster.push_back(laser_index[0]); // sart with first one 

        // loop through the indices
        for (size_t i = 1; i < laser_index.size(); ++i) {
            // check if the indices are close enought to be a cluster
            if (laser_index[i] - laser_index[i-1] < 5) {
                current_cluster.push_back(laser_index[i]);  
            }         
            else {
                clusters.push_back(current_cluster);        // save finished cluster
                current_cluster.clear();                    // start new one
                current_cluster.push_back(laser_index[i]);  
            }
        }
        
        // save the last current cluster in the all-groups vector
        if (!current_cluster.empty()) {
            clusters.push_back(current_cluster);
        }   

        return clusters;
    }

    //--------------------------------
    // Set the average of one cluster 
    //--------------------------------
    std::vector<int> average_index(const std::vector<std::vector<int>>  &clusters) {
        std::vector<int> average;
        average.reserve(clusters.size());

        // loop through each cluster
        for (const auto &clustr : clusters) {
            if (clustr.empty()) continue;      // safety guard

            long long sum = 0;                 // total sum 

            // loop through elements in clusetr
            for ( int idx : clustr) {
                sum += idx;
            }
            
            int avg = static_cast<int>(sum / static_cast<long long>(clustr.size())); // avrage index
            average.push_back(avg);  // save it
        }

        return average;
    }

    //---------------------------------
    // Get the [X,Y] of the shelf legs
    //---------------------------------
    bool index_to_xy(const sensor_msgs::msg::LaserScan &scan, 
                     int i, 
                     std::pair<double, double> &out_xy) {
 
        // 1. check index boiund
        if (i < 0 || static_cast<size_t>(i) >= scan.ranges.size()) {
            return false;
        }
        
        // 2. read the range value and check if it is valid 
        float range = scan.ranges[i];
        if (!std::isfinite(range) || range <= 0.f) {
            return false;
        }

        // 3. compute the angle of this beam 
        float theta = scan.angle_min + i * scan.angle_increment;

        // 4. convert polar(range, theta) -> cartesian(x, y)    
        out_xy = {range * std::cos(theta), range * std::sin(theta)};  


        return true;
    }

    //------------------------------
    // Tranform laser (XY) to odom 
    //------------------------------
    bool laserXY_to_odom(double lx, double ly,
                         const std::string &laser_frame,
                         double &ox, double &oy) {

        geometry_msgs::msg::PointStamped p_laser, p_odom;
        p_laser.header.stamp = this->now();
        p_laser.header.frame_id = laser_frame;
        p_laser.point.x = lx;
        p_laser.point.y = ly;
        p_laser.point.z = 0.0;

        try {
            p_odom = tf_buffer_->transform(p_laser, "odom", tf2::durationFromSec(0.2));
            ox = p_odom.point.x;
            oy = p_odom.point.y;
            return true;
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "laser->odom tranform faild: %s", ex.what());
            return false;
        }
    }

    //---------------
    // Set static TF
    //---------------
    void publish_cart_frame_static_once(double mx, double my, 
                                         const std::string &parent_frame, const std::string &child_frame, 
                                         double yaw_rad) {  

        // 1. Build the stamped transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;

        // 2. Position in parent frame coordinates
        t.transform.translation.x = mx;
        t.transform.translation.y = my;
        t.transform.translation.z = 0.0;

        // 3. Orientation yaw -> quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 5. Broadcast it
        static_tf_broadcaster_->sendTransform(t);
    }

    //-------------------
    // Drive to TF frame
    //-------------------
    bool drive_to_cart_frame(const std::string &child_frame,
                             double xy_tol = 0.05,
                             double mx_time_sec = 12.0) {

        rclcpp::Rate rate(20.0);
        const rclcpp::Time t0 = this->now();

        while (rclcpp::ok() && (this->now() - t0).seconds() < mx_time_sec ) {
            geometry_msgs::msg::TransformStamped T;

            try {
                T = tf_buffer_->lookupTransform("robot_base_footprint", child_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),1000, "TF lookup faild (base: robot_base_footprint, child: %s): %s",
                child_frame.c_str(), ex.what());
            
                rate.sleep();
                continue;
            }
            
            // How far to the TF_error
            const double ex = T.transform.translation.x;
            const double ey = T.transform.translation.y;
    
            // Distance
            const double distance = std::hypot(ex, ey);

            const double yaw_err = std::atan2(ey, ex);
            
            // Move to TF
            // stop if reache distance
            if (distance < xy_tol) {
                geometry_msgs::msg::Twist z;
                vel_pub_->publish(z);
                return true;
            }

            // porportional contol 
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = std::clamp(0.6 * ex, -0.25, 0.25);
            cmd.angular.z = std::clamp(1.5 * yaw_err, -0.25, 0.25);
            vel_pub_->publish(cmd);

            rate.sleep();
            
        }
       
        // Timeout: stop + fail        
        vel_pub_->publish(geometry_msgs::msg::Twist{});
        return false;
    }

    //------------------
    // Move foward 30cm
    //------------------
    void creep_forward(double distance_m, double speed_mps = 0.10) {
        if (distance_m <= 0.0) return;
        const double T = distance_m / std::max(1e-6, speed_mps);

        rclcpp::Rate rate(20.0);
        auto t0 = this->now();

        RCLCPP_INFO(this->get_logger(), "Moving foward 30cm.");

        while (rclcpp::ok() && (this->now() - t0).seconds() < T) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = speed_mps;
            vel_pub_->publish(cmd);
            rate.sleep();
        }

        geometry_msgs::msg::Twist z;
        vel_pub_->publish(z);
    }

    //------------
    // Lift shelf
    //------------
    void lift_shelf() {
        std_msgs::msg::String msg;
        RCLCPP_INFO(this->get_logger(), "Lifting shelf up.");
        msg.data = "up";
        elevator_up_pub_->publish(msg);
    }

    //----------
    // Service 
    //----------
    void handle_request(const std::shared_ptr<GoToLoading::Request> request,
                              std::shared_ptr<GoToLoading::Response> response) {

        // [CHECK #1] laser data is available
        if (!laser_scan_) {
            RCLCPP_WARN(this->get_logger(), "No laser data availabel.");
            response->complete = false;
            return;
        }

        // 1. Finde shelf reflective stips 
        auto shelf_laser_index = find_reflective_indices(laser_scan_); 
        RCLCPP_INFO(this->get_logger(), "reflective points = %zu", shelf_laser_index.size());

        // 2. Split laser stips in clusters of legs shelf
        auto clusters_of_legs = find_shelf_leg_cluster(shelf_laser_index);
        RCLCPP_INFO(this->get_logger(), "total shelf legs = %zu", clusters_of_legs.size());

        // [CHECK #2] if we got two shelf legs
        if (clusters_of_legs.size() < 2 ) {
            RCLCPP_WARN(this->get_logger(), "Not enough legs detected. Found %zu", clusters_of_legs.size());
            response->complete = false;
            return;
        }

        // 3. Set a average on one cluster
        auto single_index_average = average_index(clusters_of_legs);
        std::sort(single_index_average.begin(), single_index_average.end());  // order the indices (deterministic left->right)

        // [CHECK #3] average index afeter sorting
        if (single_index_average.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough centers after averaging: %zu", single_index_average.size());
            response->complete = false;
            return;
        }

        // 4. Get the [X,Y] of index 
        std::pair<double, double> p1, p2;
        bool ok1 = index_to_xy(*laser_scan_, single_index_average[0], p1);
        bool ok2 = index_to_xy(*laser_scan_, single_index_average[1], p2);
        
        // [CHECK #4] if values are usable befor TF  
        if (!ok1 || !ok2) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert to XY (ok1=%d, ok2=%d)", ok1, ok2);
            response->complete = false;
            return;
        }

        // 5. Midpoint (x,y)
        auto [x1, y1] = p1;
        auto [x2, y2] = p2;
        RCLCPP_INFO(this->get_logger(), "leg_1: x= %.2f, y= %.2f | leg_2: x= %.2f, y= %.2f", x1, y1, x2, y2);
        
        double mx = 0.5 * (x1 + x2);
        double my = 0.5 * (y1 + y2); 
        
        // 6. Set the TF at (mx, my)
        const std::string laser_frame = laser_scan_->header.frame_id;
                
        // odom x y 
        double ox, oy;

        // Tranform laser frame -> odom
        if (!laserXY_to_odom(mx, my, laser_frame, ox, oy)) {
            RCLCPP_INFO(this->get_logger(), "Failed to convert laser data to TF odom cordinates");
            response->complete = false;
            return;
        };

        const std::string child = "cart_frame";

        double yaw = std::atan2(oy,ox);

        // Set static TF
        publish_cart_frame_static_once(ox, oy, "odom", child, yaw);
        
        if (!request->attach_to_shelf) {
            RCLCPP_INFO(this->get_logger(), "final_approach=false  TF published only");
            response->complete = true;
            return;
        }

        // 7. Move to shelf
        if (!drive_to_cart_frame(child)) {
            RCLCPP_WARN(this->get_logger(), "Faild to reach cart_frame");
            response->complete = false;
            return;
        };

        // move 30cm more 
        creep_forward(0.30);
        
        // lift shelf 
        lift_shelf();

        RCLCPP_INFO(this->get_logger(), "Final approach done. Shelf lifted.");
        response->complete = true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}