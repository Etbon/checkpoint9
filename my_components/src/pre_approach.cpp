#include "my_components/pre_approach.hpp"
#include "rclcpp/utilities.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options) 
    : Node("pre_approach", options) {

    const auto cmd_topic = this->declare_parameter<std::string>("/robot/cmd_vel", "/diffbot_base_controller/cmd_vel_unstamped");
    const auto odom_topic = this->declare_parameter<std::string>("/odom","/diffbot_base_controller/odom");

    // Only creat sim time if not createt yet (my_components gaurd)
    if (!this->has_parameter("use_sim_time")) {
        (void)this->declare_parameter<bool>("use_sim_time", true);
    }

    // Initialize publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_topic, 
        10
    );
    
    // Initialze laser subsciber 
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&PreApproach::laser_callback, this, std::placeholders::_1)
    );
    
    // Initialize odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        10,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1)
    );
    
    // Initiaize timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PreApproach::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "PreApproach constructed and ready...");
}

//----------------
// Laser Callback
//----------------
void PreApproach::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Check for empty scan 
    if (msg->ranges.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 8000, "Laser ranges are empty");
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
double PreApproach::normAng(double angle) {
    while (angle > M_PI) {  // Wrap down
        angle -= 2*M_PI;    
    }
    while (angle < -M_PI) { // Wrap up 
        angle += 2*M_PI;
    }
    return angle;
}

double PreApproach::shortestAngDist(double from, double to) {
    return normAng(to - from);
}

//---------------
// Odom callback 
//---------------
void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
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
    odom_ok_ = true;   
}

//---------------------
// State Machine Logic 
//---------------------
void PreApproach::timer_callback() {
    switch (state_) {
        case State::MOVING: {
            RCLCPP_DEBUG(this->get_logger(),"State is MOVING");
               
            // 1. Check if laser disnace is > than obstecel threshold
            if (front_laser_ > OBSTACLE_THRESHOLD + 0.02) {   
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
                double radians = static_cast<double>(TARGET_DEGREES) * M_PI / 180.0;
                start_yaw_ = current_yaw_;
                goal_yaw_ = normAng(start_yaw_ + radians);

                // Change state 
                state_ = State::ROTATING;
            }
            break;
        }
        case State::ROTATING: {
            RCLCPP_DEBUG(this->get_logger(), "State is ROTATING");

            double distance_remaining  = shortestAngDist(current_yaw_, goal_yaw_);
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

                RCLCPP_INFO(this->get_logger(), "Pre-approach DONE.");
            }
            
            return;
        }
    }
}

} // namespace my_components

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach) 