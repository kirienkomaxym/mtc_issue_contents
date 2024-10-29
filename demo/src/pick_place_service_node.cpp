// pick_place_service_node.cpp

#include "mercurio_moveit_task_constructor/pick_place_service_node.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "mercurio_moveit_task_constructor/pick_place_task.h"

namespace mercurio_moveit_task_constructor {

PickPlaceServiceNode::PickPlaceServiceNode() : Node("pick_place_service_node") {
    pick_place_task_ = std::make_shared<PickPlaceTask>("pick_place_task");

    service_ = this->create_service<moveit_task_constructor_msgs::srv::PickPlace>(
        "execute_pick_place",
        std::bind(&PickPlaceServiceNode::handle_pick_place, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "PickPlaceServiceNode is ready to receive requests");
}

std::vector<double> PickPlaceServiceNode::pose_to_rpy_vector(const geometry_msgs::msg::Pose& pose) {
    std::vector<double> vec(6);
    vec[0] = pose.position.x;
    vec[1] = pose.position.y;
    vec[2] = pose.position.z;

    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    vec[3] = roll;
    vec[4] = pitch;
    vec[5] = yaw;

    return vec;
}

void PickPlaceServiceNode::handle_pick_place(
    const std::shared_ptr<moveit_task_constructor_msgs::srv::PickPlace::Request> request,
    std::shared_ptr<moveit_task_constructor_msgs::srv::PickPlace::Response> response
) {
    RCLCPP_INFO(this->get_logger(), "Received pick_place_task request");

    // Convert Pose to [x, y, z, roll, pitch, yaw]
    std::vector<double> object_pose_vec = pose_to_rpy_vector(request->object_pose);
    std::vector<double> place_pose_vec = pose_to_rpy_vector(request->place_pose);

    // Log converted poses
    RCLCPP_INFO(this->get_logger(), "Object Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                object_pose_vec[0], object_pose_vec[1], object_pose_vec[2],
                object_pose_vec[3], object_pose_vec[4], object_pose_vec[5]);

    RCLCPP_INFO(this->get_logger(), "Place Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                place_pose_vec[0], place_pose_vec[1], place_pose_vec[2],
                place_pose_vec[3], place_pose_vec[4], place_pose_vec[5]);

    // Initialize parameters
    auto param_listener = std::make_shared<pick_place_task_demo::ParamListener>(shared_from_this());
    pick_place_task_demo::Params params = param_listener->get_params();

    // Update params with new poses
    params.object_pose = object_pose_vec;
    params.place_pose = place_pose_vec;

    // Setup demo scene with updated params
    setupDemoScene(params);

    // Initialize task with new parameters
    if (!pick_place_task_->init(shared_from_this(), params)) {
        response->success = false;
        response->message = "Failed to initialize PickPlaceTask";
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PickPlaceTask");
        return;
    }

    // Plan
    if (!pick_place_task_->plan(params.max_solutions)) {
        response->success = false;
        response->message = "Failed to plan PickPlaceTask";
        RCLCPP_ERROR(this->get_logger(), "Failed to plan PickPlaceTask");
        return;
    }

    // Execute
    if (params.execute) {
        if (!pick_place_task_->execute()) {
            response->success = false;
            response->message = "Failed to execute PickPlaceTask";
            RCLCPP_ERROR(this->get_logger(), "Failed to execute PickPlaceTask");
            return;
        }
    }

    response->success = true;
    response->message = "Pick and Place task executed successfully";
    RCLCPP_INFO(this->get_logger(), "Pick and Place task executed successfully");
}

} // namespace mercurio_moveit_task_constructor
