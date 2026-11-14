#include <rclcpp/rclcpp.hpp>
#include "crane_rrt/rrt_exploration_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<gantry::RRTExplorationNode>(options);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

