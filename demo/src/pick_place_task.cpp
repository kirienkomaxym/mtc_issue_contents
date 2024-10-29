// pick_place_task.cpp
#include <Eigen/Geometry>
#include <mercurio_moveit_task_constructor/pick_place_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include "pick_place_demo_parameters.hpp"
#include <geometric_shapes/solid_primitive_dims.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mercurio_moveit_task_constructor");

namespace {
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
	return Eigen::Translation3d(values[0], values[1], values[2]) *
	       Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
	return tf2::toMsg(vectorToEigen(values));
};
}  // namespace

namespace mercurio_moveit_task_constructor {

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                 const moveit_msgs::msg::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::msg::CollisionObject createTable(const pick_place_task_demo::Params& params) {
	geometry_msgs::msg::Pose pose = vectorToPose(params.table_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = params.table_name;
	object.header.frame_id = params.table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { params.table_dimensions.at(0), params.table_dimensions.at(1),
		                                 params.table_dimensions.at(2) };
	pose.position.z -= 0.5 * params.table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createObject(const pick_place_task_demo::Params& params) {
	geometry_msgs::msg::Pose pose = vectorToPose(params.object_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = params.object_name;
	object.header.frame_id = params.object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = { params.object_dimensions.at(0), params.object_dimensions.at(1) };
	pose.position.z += 0.5 * params.object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

void setupDemoScene(const pick_place_task_demo::Params& params) {
	// Add table and object to planning scene
	rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	if (params.spawn_table)
		spawnObject(psi, createTable(params));
	spawnObject(psi, createObject(params));
}

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& params) {
	RCLCPP_INFO(LOGGER, "Initializing task pipeline");

	// Reset ROS introspection before constructing the new object
	// TODO(v4hn): global storage for Introspection services to enable one-liner
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	// Individual movement stages are collected within the Task object
	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel(node);

	/* Create planners used in various stages. Various options are available,
	   namely Cartesian, MoveIt pipeline, and joint interpolation. */
	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(1.0);
	cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	cartesian_planner->setStepSize(.01);

	// Set task properties
	t.setProperty("group", params.arm_group_name);
	t.setProperty("eef", params.eef_name);
	t.setProperty("hand", params.hand_group_name);
	t.setProperty("hand_grasping_frame", params.hand_frame);
	t.setProperty("ik_frame", params.hand_frame);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object = params.object_name](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		t.add(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Home                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(params.arm_home_pose);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool PickPlaceTask::plan(const std::size_t max_solutions) {
	RCLCPP_INFO(LOGGER, "Start searching for task solutions");

	return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTask::execute() {
	RCLCPP_INFO(LOGGER, "Executing solution trajectory");
	moveit_msgs::msg::MoveItErrorCodes execute_result;

	execute_result = task_->execute(*task_->solutions().front());

	if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
		RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
		return false;
	}

	return true;
}
}  // namespace mercurio_moveit_task_constructor
