//pick_place_demo.cpp

#include <rclcpp/rclcpp.hpp>
#include "mercurio_moveit_task_constructor/pick_place_service_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<mercurio_moveit_task_constructor::PickPlaceServiceNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}