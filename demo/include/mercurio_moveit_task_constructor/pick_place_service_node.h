// pick_place_service_node.h

#ifndef MERCURIO_MOVEIT_TASK_CONSTRUCTOR_PICK_PLACE_SERVICE_NODE_H
#define MERCURIO_MOVEIT_TASK_CONSTRUCTOR_PICK_PLACE_SERVICE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit_task_constructor_msgs/srv/pick_place.hpp>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include "pick_place_task.h"
#include "pick_place_demo_parameters.hpp"

namespace mercurio_moveit_task_constructor {

class PickPlaceTask;

class PickPlaceServiceNode : public rclcpp::Node {
public:
    PickPlaceServiceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::vector<double> pose_to_rpy_vector(const geometry_msgs::msg::Pose& pose);

    void handle_pick_place(
        const std::shared_ptr<moveit_task_constructor_msgs::srv::PickPlace::Request> request,
        std::shared_ptr<moveit_task_constructor_msgs::srv::PickPlace::Response> response
    );

    std::shared_ptr<PickPlaceTask> pick_place_task_;

    rclcpp::Service<moveit_task_constructor_msgs::srv::PickPlace>::SharedPtr service_;

    pick_place_task_demo::Params current_params_;
};

} // namespace mercurio_moveit_task_constructor

#endif // MERCURIO_MOVEIT_TASK_CONSTRUCTOR_PICK_PLACE_SERVICE_NODE_H
