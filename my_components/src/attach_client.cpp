#include "my_components/attach_client.hpp"

#include <chrono>
#include <exception>
#include <memory>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/logging.hpp"

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : rclcpp::Node("attach_client_node", options) {

    // Initilaize client
    client_ = this->create_client<GoToLoading>("/approach_shelf");
    
    // Initialize timer
    once_start_timer_ = this->create_wall_timer(std::chrono::milliseconds(300), [this](){
        if (once_start_timer_) once_start_timer_->cancel();
        call_service_when_done();
    });
};

//---------
// Client
//---------
void AttachClient::call_service_when_done() {

    // Check it the service is up 
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service /approach_shelf not availabel or time out. Requets not sende");
        return;
    }

    // Build the request 
    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = true;

    // Send the request async
    (void)client_->async_send_request(request, [this](rclcpp::Client<GoToLoading>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Final approach completed: %s", response->complete ? "true" : "false");
                
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "AttachClient response error: %s", e.what());
            }
        }
    );
};

} // namespace my_components

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient);