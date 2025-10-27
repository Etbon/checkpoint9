#pragma once

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

#include "my_components/visibility_control.h"
#include "rclcpp/node_options.hpp"

#include "attach_shelf/srv/go_to_loading.hpp"

namespace my_components {

using GoToLoading = attach_shelf::srv::GoToLoading;

class AttachServer : public rclcpp::Node { 
  public:
    explicit AttachServer(const rclcpp::NodeOptions &options);

  private:
    //------------------
    // Member variables 
    //------------------ 
    rclcpp::Service<GoToLoading>::SharedPtr server_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;

    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_; // Latest scan 

    // TF
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Motion I/O
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;

    //--------------
    // Cached data
    //--------------
    double xy_tol{0.10};
    double max_timer_sec{20.0};
    double speed_mps{0.10};

    //----------------
    // Laser callback
    //----------------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    //------------
    // Find shelf 
    //------------
    std::vector<int> find_reflective_indices(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    //-----------------
    // Find shelf legs 
    //-----------------
    std::vector<std::vector<int>> find_shelf_leg_cluster(const std::vector<int> &laser_index);

    //--------------------------------
    // Set the average of one cluster 
    //--------------------------------
    std::vector<int> average_index(const std::vector<std::vector<int>> &clusters);

    //---------------------------------
    // Get the [X,Y] of the shelf legs
    //---------------------------------
    bool index_to_xy(const sensor_msgs::msg::LaserScan &scan, int i, std::pair<double, double> &out_xy);    

    //------------------------------
    // Tranform laser (XY) to odom 
    //------------------------------
    bool laserXY_to_odom(double lx, double ly, const std::string &laser_frame, double &ox, double &oy);

    //---------------
    // Set static TF
    //---------------
    void publish_cart_frame_static_once(double mx, double my, const std::string &parent_frame, const std::string &child_frame, double yaw_rad);

    //-------------------
    // Drive to TF frame
    //-------------------
    bool drive_to_cart_frame(const std::string &child_frame, double xy_tol, double max_time_sec);

    //------------------
    // Move foward 30cm
    //------------------
    void creep_forward(double distance_m, double speed_mps, double max_time_sec);

    //------------
    // Lift shelf
    //------------
    void lift_shelf();

    //----------
    // Shutdown 
    //----------
    void graceful_shutdown_after(std::chrono::milliseconds delay);

    //----------
    // Service 
    //----------
    void handle_request(const std::shared_ptr<GoToLoading::Request> request, 
                              std::shared_ptr<GoToLoading::Response> response);
};

} // namespace my_components