#include <algorithm>
#include <cmath>
#include <limits> 
#include <memory>   
#include <chrono>   
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/logging.hpp"


using GoToLoading = attach_shelf::srv::GoToLoading;

class PreApproach : public rclcpp::Node {
  public:
    PreApproach()
        : Node("pre_approach_node") {

        // Parameter declaration
        this->declare_parameter<double>("obstacle", 0.3);
        this->declare_parameter<int>("degrees", -90);
        this->declare_parameter<bool>("final_approach", false);
    
        // Parameter retirval 
        this->get_parameter("obstacle", obstacle_threshold_);
        this->get_parameter("degrees", target_degrees_);
        this->get_parameter("final_approach", attach_to_shelf_);
    
        // Initialize the state machine 
        state_ = State::MOVING;

        // Initialize client 
        client_ = this->create_client<GoToLoading>("/approach_shelf");

        // Initialize publisher 
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot/cmd_vel", 10
        );

        // Initialize laser subscriber
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&PreApproach::laser_callback, this, std::placeholders::_1)
        );

        // Initiliaze odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&PreApproach::odom_callback, this, std::placeholders::_1)
        );

        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PreApproach::timer_callback, this)
        );
    }
  private:
    //-------------------------
    // Define member variables
    //-------------------------
    rclcpp::Client<GoToLoading>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Front laser
    float front_laser_ = std::numeric_limits<float>::infinity();

    // Odometry
    bool odom_ok_{false};
    double current_yaw_{0.0};
    double start_yaw{0.0};
    double goal_yaw{0.0};
    
    // Rotation 
    double obstacle_threshold_;
    int target_degrees_;

    // Cient bool 
    bool attach_to_shelf_{false};

    // Call once flag
    bool finished_{false};  

    //--------
    // Client   
    //--------
    void call_service_when_done(bool attach) {
        // Wait for server
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /approach_shelf...");
        }        

        // Create request 
        auto request = std::make_shared<GoToLoading::Request>();
        request->attach_to_shelf = attach;

        // Async send lambda
        client_->async_send_request(request, [this](rclcpp::Client<GoToLoading>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Service reply complete=%s", response->complete ? "true" : "false");

                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service future error: %s", e.what());
                }
                
                // Shutting down
                rclcpp::shutdown();
            }
        );         
        
        RCLCPP_INFO(this->get_logger(), "Request sent (attach_to_shelf=%s). Waiting for server reply...", 
            attach ? "true" : "false"
        );
    }
    
    //-----------------
    // Laser Callback 
    //-----------------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check for empty scan 
        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Laser ranges are empty");
            return;
        }
        
        // Index
        int front_index = static_cast<int>(std::lround((0.0 - msg->angle_min) / msg->angle_increment));

        // Clamp it in the range 
        front_index = std::clamp(front_index, 0, static_cast<int>(msg->ranges.size()) - 1);

        // Find the index for the front
        float distance_at_front = msg->ranges[front_index]; 

        // Filter NaN & Inf
        if (std::isnan(distance_at_front) || std::isinf(distance_at_front)) {
            RCLCPP_WARN(this->get_logger(), "Front laser reading is invalid!");
            return;
        }

        // Seve the current scan out of the function
        front_laser_ = distance_at_front;
    }

    // Odom callback helper 
    static double normAng(double angle) {
        while (angle > M_PI) {  // Wrap down
            angle -= 2*M_PI;    
        }
        while (angle < -M_PI) { // Wrap up 
            angle += 2*M_PI;
        }
        return angle;
    } 

    static double shortestAngDist(double from, double to) {
        return normAng(to - from);
    }

    //---------------
    // Odom callback
    //---------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // Get the position from odometry 
        const geometry_msgs::msg::Quaternion &q_msg = odom_msg->pose.pose.orientation;
        
        tf2::Quaternion q_tf;                             // make TF2 quaternion
        tf2::fromMsg(q_msg, q_tf);                        // convert message -> TF2

        if (q_tf.length2() > 0.0) {                       // normalize 
            q_tf.normalize();
        }

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);    // get roll,pitch, yaw

        current_yaw_ = normAng(yaw);                      // keep yaw in [-pi, pi]
        odom_ok_ = true;                                  // make odom valid  
    }
    

    // State machine definition 
    enum class State {MOVING, ROTATING, FINISH};
    State state_; // Current robot state
    
    //-----------------------
    //  State Machine Logic
    //-----------------------
    void timer_callback() {
        switch (state_) {
            case State::MOVING: {
                RCLCPP_INFO(this->get_logger(),"State is MOVING");
               
                // 1. Check if laser disnace is > than obstecel threshold
                if (front_laser_ > obstacle_threshold_+ 0.02) {   
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = 0.2;
                    twist_msg.angular.z = 0.0;
                    publisher_->publish(twist_msg);
                }
                else {
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    publisher_->publish(twist_msg);

                    // Guard: if odom data is not ready skip 
                    if (!odom_ok_) {
                        return;
                    }
                
                    // Compute the target rotation 
                    double radians = static_cast<double>(target_degrees_) * M_PI / 180.0;
                    start_yaw = current_yaw_;
                    goal_yaw = normAng(start_yaw + radians);

                    // Change state 
                    state_ = State::ROTATING;
                }
                break;
            }
            case State::ROTATING: {
                RCLCPP_INFO(this->get_logger(), "State is ROTATING");

                double distance_remaining  = shortestAngDist(current_yaw_, goal_yaw);
                const double tolerance = 0.02;

                if (std::fabs(distance_remaining) > tolerance) {
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = (distance_remaining > 0.0) ? 0.2 : -0.2;
                    publisher_->publish(twist_msg);
                }
                else {
                    geometry_msgs::msg::Twist twist_msg;  // zeros
                    publisher_->publish(twist_msg);
                    state_ = State::FINISH;               // change state
                }
                
                break;
            }
            case State::FINISH: {
                if (!finished_) {
                    finished_ = true;

                    RCLCPP_DEBUG(this->get_logger(), "State is FINISH");

                    // Stop 
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    publisher_->publish(twist_msg);

                    // Stop the timer
                    timer_->cancel();

                    RCLCPP_INFO(this->get_logger(), "Pre-approach DONE. Calling the server...");
                    
                    // Calling the server 
                    call_service_when_done(attach_to_shelf_);
                }

                return;
            }
        }
    }
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproach>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}