// 中文说明：提供 `/trajectory_retime/retime` 服务的 ROS 2 C++ 节点。
// 节点接收 JointTrajectory 和速度/加速度缩放，调用 MoveIt 时间参数化后返回新轨迹；
// 它不直接发送控制器命令，调用方仍负责后续执行与安全检查。
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_retime_server/srv/retime_trajectory.hpp"
#include "trajectory_retime_server/validation.hpp"

namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

bool finite_values(const std::vector<double> & values)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

class TrajectoryRetimeServer
{
public:
  explicit TrajectoryRetimeServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    std::string service_name = "/retime_trajectory";
    if (node_->has_parameter("service_name")) {
      service_name = node_->get_parameter("service_name").as_string();
    } else {
      service_name = node_->declare_parameter<std::string>("service_name", service_name);
    }
    if (service_name.empty()) {
      throw std::invalid_argument("Parameter 'service_name' must not be empty");
    }

    load_robot_model();

    service_ = node_->create_service<trajectory_retime_server::srv::RetimeTrajectory>(
      service_name,
      std::bind(
        &TrajectoryRetimeServer::handle, this, std::placeholders::_1,
        std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Trajectory retime service ready on '%s'", service_name.c_str());
  }

private:
  bool has_nonempty_string_parameter(const std::string & name) const
  {
    if (!node_->has_parameter(name)) {
      return false;
    }
    const auto parameter = node_->get_parameter(name);
    return parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
           !parameter.as_string().empty();
  }

  bool has_kinematics_parameters() const
  {
    const auto result = node_->list_parameters({"robot_description_kinematics"}, 10);
    return !result.names.empty() || !result.prefixes.empty();
  }

  void load_robot_model()
  {
    if (!has_nonempty_string_parameter("robot_description")) {
      model_error_ = "Parameter 'robot_description' is missing or empty on this node.";
    } else if (!has_nonempty_string_parameter("robot_description_semantic")) {
      model_error_ = "Parameter 'robot_description_semantic' is missing or empty on this node.";
    } else if (!has_kinematics_parameters()) {
      model_error_ = "Parameter tree 'robot_description_kinematics' is missing on this node.";
    }

    if (!model_error_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "%s", model_error_.c_str());
      return;
    }

    try {
      robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
      robot_model_ = robot_model_loader_->getModel();
    } catch (const std::exception & error) {
      model_error_ = std::string("RobotModel loading failed: ") + error.what();
      RCLCPP_ERROR(node_->get_logger(), "%s", model_error_.c_str());
      return;
    }

    if (!robot_model_) {
      model_error_ = "RobotModel loading failed; injected URDF or SRDF is invalid.";
      RCLCPP_ERROR(node_->get_logger(), "%s", model_error_.c_str());
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded robot model '%s'", robot_model_->getName().c_str());
  }

  void fail(
    const std::string & message,
    const std::shared_ptr<trajectory_retime_server::srv::RetimeTrajectory::Response> & response) const
  {
    response->success = false;
    response->message = message;
    response->retimed = trajectory_msgs::msg::JointTrajectory();
    RCLCPP_WARN(node_->get_logger(), "Trajectory retiming rejected: %s", message.c_str());
  }

  void handle(
    const std::shared_ptr<trajectory_retime_server::srv::RetimeTrajectory::Request> request,
    std::shared_ptr<trajectory_retime_server::srv::RetimeTrajectory::Response> response)
  {
    response->success = false;
    response->message.clear();
    response->retimed = trajectory_msgs::msg::JointTrajectory();

    if (!robot_model_) {
      fail(model_error_.empty() ? "RobotModel is not loaded." : model_error_, response);
      return;
    }
    if (!trajectory_retime_server::validation::valid_scaling(request->velocity_scaling)) {
      fail("velocity_scaling must be finite and in the interval (0, 1].", response);
      return;
    }
    if (!trajectory_retime_server::validation::valid_scaling(request->acceleration_scaling)) {
      fail("acceleration_scaling must be finite and in the interval (0, 1].", response);
      return;
    }
    if (request->group_name.empty()) {
      fail("group_name must not be empty.", response);
      return;
    }

    const auto * joint_model_group = robot_model_->getJointModelGroup(request->group_name);
    if (joint_model_group == nullptr) {
      fail("JointModelGroup not found: " + request->group_name, response);
      return;
    }

    const auto & input = request->trajectory;
    if (input.points.size() < 2 || input.joint_names.empty()) {
      fail("Input trajectory must contain joint names and at least two points.", response);
      return;
    }
    if (!trajectory_retime_server::validation::unique_nonempty_names(input.joint_names)) {
      fail("Input trajectory contains an empty or duplicate joint name.", response);
      return;
    }

    const auto group_joint_names = joint_model_group->getVariableNames();
    if (input.joint_names.size() != group_joint_names.size()) {
      fail("Input trajectory joint set must exactly match the selected planning group.", response);
      return;
    }

    std::unordered_map<std::string, std::size_t> input_index;
    input_index.reserve(input.joint_names.size());
    for (std::size_t index = 0; index < input.joint_names.size(); ++index) {
      input_index.emplace(input.joint_names[index], index);
    }
    for (const auto & joint_name : group_joint_names) {
      if (input_index.find(joint_name) == input_index.end()) {
        fail("Input trajectory is missing group joint: " + joint_name, response);
        return;
      }
    }

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    robot_trajectory::RobotTrajectory robot_trajectory(robot_model_, joint_model_group);
    constexpr double nominal_waypoint_duration = 0.01;

    for (std::size_t point_index = 0; point_index < input.points.size(); ++point_index) {
      const auto & point = input.points[point_index];
      if (point.positions.size() != input.joint_names.size() || !finite_values(point.positions)) {
        fail("Every input point must contain one finite position per joint.", response);
        return;
      }

      for (const auto & joint_name : group_joint_names) {
        state.setVariablePosition(joint_name, point.positions[input_index.at(joint_name)]);
      }
      state.update();
      if (!state.satisfiesBounds(joint_model_group)) {
        fail("Input point " + std::to_string(point_index) + " violates joint position bounds.", response);
        return;
      }
      robot_trajectory.addSuffixWayPoint(
        state, point_index == 0 ? 0.0 : nominal_waypoint_duration);
    }

    try {
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      if (!totg.computeTimeStamps(
          robot_trajectory, request->velocity_scaling, request->acceleration_scaling))
      {
        fail("TOTG computeTimeStamps() failed.", response);
        return;
      }
    } catch (const std::exception & error) {
      fail(std::string("TOTG threw an exception: ") + error.what(), response);
      return;
    }

    moveit_msgs::msg::RobotTrajectory output_message;
    robot_trajectory.getRobotTrajectoryMsg(output_message);
    auto output = std::move(output_message.joint_trajectory);
    output.header = input.header;

    if (output.points.size() < 2 || output.joint_names.size() != input.joint_names.size()) {
      fail("TOTG returned an incomplete trajectory.", response);
      return;
    }

    std::unordered_map<std::string, std::size_t> output_index;
    output_index.reserve(output.joint_names.size());
    for (std::size_t index = 0; index < output.joint_names.size(); ++index) {
      if (!output_index.emplace(output.joint_names[index], index).second) {
        fail("TOTG returned duplicate joint names.", response);
        return;
      }
    }
    for (const auto & joint_name : input.joint_names) {
      if (output_index.find(joint_name) == output_index.end()) {
        fail("TOTG output is missing joint: " + joint_name, response);
        return;
      }
    }

    double previous_time = -1.0;
    for (auto & point : output.points) {
      if (point.positions.size() != output.joint_names.size() || !finite_values(point.positions)) {
        fail("TOTG returned invalid joint positions.", response);
        return;
      }
      if ((!point.velocities.empty() &&
        (point.velocities.size() != output.joint_names.size() || !finite_values(point.velocities))) ||
        (!point.accelerations.empty() &&
        (point.accelerations.size() != output.joint_names.size() ||
        !finite_values(point.accelerations))))
      {
        fail("TOTG returned invalid velocity or acceleration data.", response);
        return;
      }

      const double current_time = duration_seconds(point.time_from_start);
      if (current_time < 0.0 || current_time <= previous_time) {
        fail("TOTG returned non-increasing time_from_start values.", response);
        return;
      }
      previous_time = current_time;

      std::vector<double> reordered_positions(input.joint_names.size());
      std::vector<double> reordered_velocities;
      std::vector<double> reordered_accelerations;
      if (!point.velocities.empty()) {
        reordered_velocities.resize(input.joint_names.size());
      }
      if (!point.accelerations.empty()) {
        reordered_accelerations.resize(input.joint_names.size());
      }

      for (std::size_t index = 0; index < input.joint_names.size(); ++index) {
        const auto source_index = output_index.at(input.joint_names[index]);
        reordered_positions[index] = point.positions[source_index];
        if (!point.velocities.empty()) {
          reordered_velocities[index] = point.velocities[source_index];
        }
        if (!point.accelerations.empty()) {
          reordered_accelerations[index] = point.accelerations[source_index];
        }
      }
      point.positions = std::move(reordered_positions);
      if (!point.velocities.empty()) {
        point.velocities = std::move(reordered_velocities);
      }
      if (!point.accelerations.empty()) {
        point.accelerations = std::move(reordered_accelerations);
      }
      point.effort.clear();
    }
    output.joint_names = input.joint_names;

    response->retimed = std::move(output);
    response->success = true;
    response->message =
      "OK, total_time_sec=" + std::to_string(previous_time) +
      ", vel_scale=" + std::to_string(request->velocity_scaling) +
      ", acc_scale=" + std::to_string(request->acceleration_scaling);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<trajectory_retime_server::srv::RetimeTrajectory>::SharedPtr service_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::string model_error_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("trajectory_retime_server", options);
  auto server = std::make_shared<TrajectoryRetimeServer>(node);

  rclcpp::spin(node);
  server.reset();
  rclcpp::shutdown();
  return 0;
}
