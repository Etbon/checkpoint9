#pragma once

#include <rclcpp/rclcpp.hpp>
#include "attach_shelf/srv/go_to_loading.hpp"

namespace my_components {

using GoToLoading = attach_shelf::srv::GoToLoading;

class AttachClient : public rclcpp::Node {
  public:
    explicit AttachClient(const rclcpp::NodeOptions &options);

  private:
    //------------------
    // Member variables
    //------------------
    rclcpp::Client<GoToLoading>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr once_start_timer_;

    //--------
    // Client
    //--------
    void call_service_when_done();
    
};

} // namespace my_components