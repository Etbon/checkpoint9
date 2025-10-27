#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "my_components/attach_server.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"

int main(int argc, char**argv ) {
    rclcpp::init(argc,argv);
    
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true); // Set the simulation time

    auto node = std::make_shared<my_components::AttachServer>(options);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();    
    return 0;
}