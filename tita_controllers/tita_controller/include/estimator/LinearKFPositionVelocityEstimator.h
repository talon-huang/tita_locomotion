/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef LINEAR_KF_POSITION_VELOCITY_ESTIMATOR_H_
#define LINEAR_KF_POSITION_VELOCITY_ESTIMATOR_H_

#include "estimator/StateEstimatorContainer.h"
#define USE_TOPIC 0

#if USE_TOPIC
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
class LinearKFPositionVelocityEstimator : public GenericEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();

protected:
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

private:
  vector_t feetHeights_;
  size_t numContacts_, dimContacts_, numState_, numObserve_;
  matrix_t a_, b_, c_, q_, p_, r_;
  vector_t xHat_, ps_, vs_;
#if USE_TOPIC
  void odom_cb();
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  void gpsPositionCb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void gpsSpeedCb(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gps_speed_sub_;

  std::shared_ptr<geometry_msgs::msg::PointStamped> gps_position_msg_;
  std::shared_ptr<geometry_msgs::msg::Vector3> gps_speed_msg_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
#endif
};

#endif  // LINEAR_KF_POSITION_VELOCITY_ESTIMATOR_H_
