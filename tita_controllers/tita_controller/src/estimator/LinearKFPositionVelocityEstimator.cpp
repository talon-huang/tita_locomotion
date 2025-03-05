// // Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include "estimator/LinearKFPositionVelocityEstimator.h"

// /*!
//  * Initialize the state estimator
//  */

// void LinearKFPositionVelocityEstimator::setup()
// {
//   numContacts_ = 2;
//   dimContacts_ = 3 * numContacts_;                // 6
//   numState_ = 6 + dimContacts_;                   // 12
//   numObserve_ = 2 * dimContacts_ + numContacts_;  // 14

//   xHat_.setZero(numState_);
//   ps_.setZero(dimContacts_);
//   vs_.setZero(dimContacts_);
//   a_.setIdentity(numState_, numState_);
//   b_.setZero(numState_, 3);
//   matrix_t c1(3, 6), c2(3, 6);
//   c1 << matrix3_t::Identity(), matrix3_t::Zero();
//   c2 << matrix3_t::Zero(), matrix3_t::Identity();
//   c_.setZero(numObserve_, numState_);
//   for (size_t i = 0; i < numContacts_; ++i) {
//     c_.block(3 * i, 0, 3, 6) = c1;
//     c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2;
//     c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;
//   }
//   c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

//   q_.setIdentity(numState_, numState_);
//   p_ = 100. * q_;
//   r_.setIdentity(numObserve_, numObserve_);
//   feetHeights_.setZero(numContacts_);
//   xHat_[2] = 0.080;
// }

// #if USE_TOPIC

// LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator()
// {
//   node_ = rclcpp::Node::make_shared("linear_kalman_filter_node");
//   executor_.add_node(node_);
//   std::thread([this]() { executor_.spin(); }).detach();
//   auto subscribers_qos = rclcpp::SystemDefaultsQoS();
//   subscribers_qos.keep_last(1);
//   subscribers_qos.best_effort();

//   gps_position_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
//     "tita_webots/gps", subscribers_qos,
//     std::bind(&LinearKFPositionVelocityEstimator::gpsPositionCb, this, std::placeholders::_1));

//   gps_speed_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
//     "tita_webots/gps/speed_vector", subscribers_qos,
//     std::bind(&LinearKFPositionVelocityEstimator::gpsSpeedCb, this, std::placeholders::_1));

//   gps_position_msg_ = std::make_shared<geometry_msgs::msg::PointStamped>();
//   gps_speed_msg_ = std::make_shared<geometry_msgs::msg::Vector3>();
//   odom_publisher_ =
//     node_->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());
//   odom_timer_ = node_->create_wall_timer(
//     std::chrono::milliseconds(100), std::bind(&LinearKFPositionVelocityEstimator::odom_cb, this));
//   odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
// }

// void LinearKFPositionVelocityEstimator::odom_cb()
// {
//   std::string frame_prefix = std::string(node_->get_namespace()) + "/";

//   // 发布TransformStamped消息
//   geometry_msgs::msg::TransformStamped odom_trans;
//   odom_trans.header.stamp = node_->now();
//   odom_trans.header.frame_id = "odom";
//   odom_trans.child_frame_id = frame_prefix + this->_stateEstimatorData.parameters->base_name;
//   odom_trans.transform.translation.x = this->_stateEstimatorData.result->position(X);
//   odom_trans.transform.translation.y = this->_stateEstimatorData.result->position(Y);
//   odom_trans.transform.translation.z = this->_stateEstimatorData.result->position(Z);
//   odom_trans.transform.rotation = tf2::toMsg(tf2::Quaternion(
//     this->_stateEstimatorData.result->orientation(QX), this->_stateEstimatorData.result->orientation(QY),
//     this->_stateEstimatorData.result->orientation(QZ), this->_stateEstimatorData.result->orientation(QW)));
//   odom_broadcaster_->sendTransform(odom_trans);

//   // 发布Odometry消息
//   nav_msgs::msg::Odometry odom_msg;
//   odom_msg.header.stamp = node_->now();
//   odom_msg.header.frame_id = "odom";
//   odom_msg.child_frame_id = frame_prefix + this->_stateEstimatorData.parameters->base_name;
//   odom_msg.pose.pose.position.x = this->_stateEstimatorData.result->position(X);
//   odom_msg.pose.pose.position.y = this->_stateEstimatorData.result->position(Y);
//   odom_msg.pose.pose.position.z = this->_stateEstimatorData.result->position(Z);
//   odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
//     this->_stateEstimatorData.result->orientation(QX), this->_stateEstimatorData.result->orientation(QY),
//     this->_stateEstimatorData.result->orientation(QZ), this->_stateEstimatorData.result->orientation(QW)));
//   odom_msg.twist.twist.linear.x = this->_stateEstimatorData.result->vWorld(X);
//   odom_msg.twist.twist.linear.y = this->_stateEstimatorData.result->vWorld(Y);
//   odom_msg.twist.twist.linear.z = this->_stateEstimatorData.result->vWorld(Z);
//   odom_msg.twist.twist.angular.x = this->_stateEstimatorData.result->omegaWorld(X);
//   odom_msg.twist.twist.angular.y = this->_stateEstimatorData.result->omegaWorld(Y);
//   odom_msg.twist.twist.angular.z = this->_stateEstimatorData.result->omegaWorld(Z);
//   odom_msg.twist.covariance = {};
//   odom_publisher_->publish(odom_msg);
// }

// void LinearKFPositionVelocityEstimator::gpsPositionCb(
//   const geometry_msgs::msg::PointStamped::SharedPtr msg)
// {
//   *gps_position_msg_ = std::move(*msg);
// }

// void LinearKFPositionVelocityEstimator::gpsSpeedCb(const geometry_msgs::msg::Vector3::SharedPtr msg)
// {
//   *gps_speed_msg_ = std::move(*msg);
// }

// // void LinearKFPositionVelocityEstimator::run()
// // {
// //   this->_stateEstimatorData.result->position =
// //     vector3_t{gps_position_msg_->point.x, gps_position_msg_->point.y, gps_position_msg_->point.z};
// //   this->_stateEstimatorData.result->vWorld =
// //     vector3_t{gps_speed_msg_->x, gps_speed_msg_->y, gps_speed_msg_->z};
// //   this->_stateEstimatorData.result->vBody =
// //     this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
// // }

// #else
// LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator() {}
// #endif
// void LinearKFPositionVelocityEstimator::run()
// {
//   imuProcessNoisePosition_ = this->_stateEstimatorData.parameters->imu_process_noise_position;
//   imuProcessNoiseVelocity_ = this->_stateEstimatorData.parameters->imu_process_noise_velocity;
//   footProcessNoisePosition_ = this->_stateEstimatorData.parameters->foot_process_noise_position;
//   footSensorNoisePosition_ = this->_stateEstimatorData.parameters->imu_sensor_noise_position;
//   footSensorNoiseVelocity_ = this->_stateEstimatorData.parameters->imu_sensor_noise_velocity;
//   footHeightSensorNoise_ = this->_stateEstimatorData.parameters->foot_sensor_noise_position;

//   scalar_t dt = this->_stateEstimatorData.parameters->dt;
//   a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
//   b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
//   b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();
//   q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
//   q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();
//   q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

//   matrix_t q = matrix_t::Identity(numState_, numState_);
//   q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
//   q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
//   q.block(6, 6, dimContacts_, dimContacts_) =
//     q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;

//   matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
//   r.block(0, 0, dimContacts_, dimContacts_) =
//     r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
//   r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) =
//     r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;
//   r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
//     r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) *
//     footHeightSensorNoise_;
//   std::vector<vector3_t> eePos, eeVel;

//   // matrix3_t Rbod = this->_stateEstimatorData.result->rBody.transpose();
//   for (auto index = 0; index < numContacts_; index++) {
//     eePos.push_back(this->_stateEstimatorData.legWheelsData->contact_position_world[index] - this->_stateEstimatorData.result->position);
//     eeVel.push_back(this->_stateEstimatorData.legWheelsData->contact_velocity_world[index] - this->_stateEstimatorData.result->vWorld);
//   }
  
//   for (size_t i = 0; i < numContacts_; i++) {
//     int i1 = 3 * i;

//     int qIndex = 6 + i1;
//     int rIndex1 = i1;
//     int rIndex2 = dimContacts_ + i1;
//     int rIndex3 = 2 * dimContacts_ + i;
//     bool isContact = true;  // contactFlag_[i];

//     scalar_t high_suspect_number(100);
//     q.block(qIndex, qIndex, 3, 3) =
//       (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
//     r.block(rIndex1, rIndex1, 3, 3) =
//       (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
//     r.block(rIndex2, rIndex2, 3, 3) =
//       (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
//     r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

//     ps_.segment(3 * i, 3) = -eePos[i];
//     // ps_.segment(3 * i, 3)[2] += footRadius_;
//     vs_.segment(3 * i, 3) = -eeVel[i];
//     feetHeights_(i) = eePos[i](2);// + xHat_[2];
//   }
//   vector3_t g(0, 0, -9.81);
//   vector3_t accel = this->_stateEstimatorData.result->aWorld + g;
//   vector_t y(numObserve_);
//   y << ps_, vs_, feetHeights_;
//   // predict
//   xHat_ = a_ * xHat_ + b_ * accel;
//   matrix_t at = a_.transpose();
//   matrix_t pm = a_ * p_ * at + q;
//   // update
//   matrix_t cT = c_.transpose();
//   matrix_t yModel = c_ * xHat_;
//   matrix_t ey = y - yModel;       // 残差 residual
//   matrix_t s = c_ * pm * cT + r;  // 测量残差协方差
//   vector_t sEy = s.lu().solve(ey);
//   xHat_ += pm * cT * sEy;
//   matrix_t sC = s.lu().solve(c_);
//   p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * sC) * pm;

//   matrix_t pt = p_.transpose();
//   p_ = (p_ + pt) / 2.0;
//   // PRINT_MAT(p_);
//   // this->_stateEstimatorData.result->position =
//   //   vector3_t{gps_position_msg_->point.x, gps_position_msg_->point.y, gps_position_msg_->point.z};
//   // this->_stateEstimatorData.result->vWorld =
//   //   vector3_t{gps_speed_msg_->x, gps_speed_msg_->y, gps_speed_msg_->z};
//   // this->_stateEstimatorData.result->vBody =
//   //   this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
//   // std::cout << "real_kf_position   " <<  this->_stateEstimatorData.result->position.transpose() << std::endl;
//   // std::cout << "real_kf_vWorld     " <<  this->_stateEstimatorData.result->vWorld.transpose() << std::endl;
//   // std::cout << "real_kf_vBody      " <<  this->_stateEstimatorData.result->vBody.transpose() << std::endl;

//   this->_stateEstimatorData.result->position = xHat_.segment<3>(0);
//   this->_stateEstimatorData.result->position(2) = -(feetHeights_(0) + feetHeights_(1))/2.0;

//   this->_stateEstimatorData.result->vWorld = xHat_.segment<3>(3);
//   this->_stateEstimatorData.result->vBody =
//     this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
//   // std::cout << "linear_kf_position " <<  this->_stateEstimatorData.result->position.transpose() << std::endl;
//   // std::cout << "linear_kf_vWorld   " <<  this->_stateEstimatorData.result->vWorld.transpose() << std::endl;
//   // std::cout << "linear_kf_vBody    " <<  this->_stateEstimatorData.result->vBody.transpose() << std::endl;


// }
