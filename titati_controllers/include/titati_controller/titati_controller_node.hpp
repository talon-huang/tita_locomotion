// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TITATI_CONTROLLER__TITATI_CONTROLLER_NODE_HPP_
#define TITATI_CONTROLLER__TITATI_CONTROLLER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
// auto-generated by generate_parameter_library
#include "titati_controller_parameters.hpp"

// #include "controller_fsm/FSM.h"
// #include "controller_common/Benchmark.h"
// #include "controller_estimator/LinearKFPositionVelocityEstimator.h"
// #include "controller_estimator/OrientationEstimator.h"

#include "tita_utils/topic_names.hpp"
#include "task/Task.hpp"
namespace titati_locomotion
{
using LoanedCommandInterface = std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
using LoanedStateInterface = std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
struct Joint
{
  Joint() {}
  LoanedCommandInterface position_command_handle;
  LoanedCommandInterface velocity_command_handle;
  LoanedCommandInterface effort_command_handle;

  LoanedStateInterface position_handle;
  LoanedStateInterface velocity_handle;
  LoanedStateInterface effort_handle;
  std::string name;
};

class TitatiController : public controller_interface::ControllerInterface
{
public:
  ~TitatiController();
  TitatiController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void fsm_goal_cb(const std_msgs::msg::String::SharedPtr msg);
  void odom_cb();
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_goal_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  
protected:
  std::vector<std::shared_ptr<Joint>> joints_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> sensor_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;
  std::shared_ptr<titati_controller::ParamListener> param_listener_;
  titati_controller::Params params_;

protected: // TODO:
  void setup_controller();
  void setup_control_parameters();
  virtual void setup_state_estimate();
  void update_control_parameters();

  int init_estimator_count_{0};
  // std::shared_ptr<FSM> FSMController_;
  // std::shared_ptr<ControlFSMData> controlData_;
  void mainLoopThread();
  // void lqrLoopThread();
  // std::thread main_thread_;//, lqr_thread_;
  // std::atomic<bool> main_thread_running_{false};//, lqr_thread_running_{false};
  // std::mutex mutex_;
  // benchmark::RepeatedTimer mpcTimer_;
  // benchmark::RepeatedTimer wbcTimer_;
  std::shared_ptr<WheelQuadruped_Task> robot_task_;
  bool is_running_ = false;

  float accl_[3];
  float gyro_[3];
  float quat_[4];
  float motor_pos_[16];
  float motor_vel_[16];

  float torque_[16];
  int mode_ = 0;
  VelGaitCmd vel_gait_cmd_;
  bool init_flag_ = false;

};

class CheaterTitatiController : public TitatiController
{
public:
  void setup_state_estimate() override;
};
}  // namespace tita_locomotion
#endif  // TITATI_CONTROLLER__TITATI_CONTROLLER_NODE_HPP_
