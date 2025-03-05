/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-06-01 13:45:56
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-06-14 11:24:35
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/include/incestimator/PositionVelocityEstimator.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "incestimator/StateEstimatorContainer.h"
#include "common/orientation_tools.h"
#include "common/LegData.h"
#include "common/AttitudeData.h"
/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();
  void set(LegData* legdata_, AttitudeData* attidata_) {
    legdata = legdata_;
    attidata = attidata_;
  };
  void set_contact(const Vec4<float> contact_) {
    contact_schedule = contact_;
  }

 private:
  Eigen::Matrix<T, 18, 1> _xhat;
  Eigen::Matrix<T, 12, 1> _ps;
  Eigen::Matrix<T, 12, 1> _vs;
  Eigen::Matrix<T, 18, 18> _A;
  Eigen::Matrix<T, 18, 18> _Q0;
  Eigen::Matrix<T, 18, 18> _P;
  Eigen::Matrix<T, 28, 28> _R0;
  Eigen::Matrix<T, 18, 3> _B;
  Eigen::Matrix<T, 28, 18> _C;
  LegData* legdata;
  AttitudeData* attidata;
  Vec4<float> contact_schedule;
  // Vec4<float> qd_wheel_tmp;
};


#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
