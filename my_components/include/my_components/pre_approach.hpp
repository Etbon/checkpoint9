#pragma once

#include "my_components/visibility_control.h"

#include <limits>
#include <algorithm>
#include <cmath>
#include <limits> 
#include <memory>   
#include <chrono>   

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace my_components {

class PreApproach : public rclcpp::Node {
  public:
    explicit PreApproach(const rclcpp::NodeOptions &options);
    
  private:
    //------------------
    // Member variables 
    //------------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 
    rclcpp::TimerBase::SharedPtr timer_;
    
    //--------------------
    // State machine enum
    //--------------------
    enum class State {MOVING, ROTATING, FINISH};
    State state_{State::MOVING};

    //-------------
    // Cached data
    //-------------
    float front_laser_{std::numeric_limits<float>::infinity()};
    bool odom_ok_{false};
    double current_yaw_{0.0};
    double start_yaw_{0.0};
    double goal_yaw_{0.0};
    bool finished_{false};

    //----------
    // Behavior 
    //----------
    static constexpr double OBSTACLE_THRESHOLD{0.30};
    static constexpr int TARGET_DEGREES{-90};

    //-----------
    // Callbacks
    //-----------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void timer_callback();

    //--------------
    // Math helpers
    //--------------
    static double normAng(double angle);
    static double shortestAngDist(double from, double to);
};

} // namespace my_componets

