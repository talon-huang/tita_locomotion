/*
 * humanoid_state_publisher - a ros package to generate necessary data for humanoid robots
 *
 * Copyright 2017-2020 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PINWRAP_H__
#define __PINWRAP_H__

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>
#include <vector>

#include "common/Math/orientation_tools.h"
#include "common/enumClass.h"
// #include "common/StateEstimate.h"
#include "common/RobotParameters.h"
#include "common/Utilities/pseudoInverse.h"
#include "planner/DesiredStateCommand.h"
// namespace tita_controller {
#include <qpOASES.hpp>

#include "lqr/riccati_solver.h"
#include "wbc/HoQp.h"
#include "wbc/Task.h"
struct RigidBody
{
  scalar_t mass;
  vector3_t com;
  matrix3_t inertia;
  pinocchio::SE3 transform;
  RigidBody()
  {
    mass = 0.0;
    com = vector3_t::Zero();
    inertia = matrix3_t::Zero();
    transform = pinocchio::SE3::Identity();
  }
};
class PinocchioInterface
{
public:
  PinocchioInterface(
    std::shared_ptr<RobotControlParameters> param, std::shared_ptr<LowlevelState> low_state,
    std::shared_ptr<LowlevelCmd> low_command, std::shared_ptr<StateEstimate> state_estimate,
    std::shared_ptr<DesiredStateCommand> state_command,
    std::shared_ptr<WheelLeggedData> wheel_legged_data);
  void setupPinocchioInterface(const bool & verbose = false);
  void updateState();
  void updateWBC();
  std::shared_ptr<RigidBody> rigid_body_data_;

private:
  void updateWeightedWbc();
  void updateHierarchicalWbc();
  Task formulateFloatingBaseEomTask();
  Task formulateNoContactMotionTask();
  Task formulateLinearTask();
  Task formulateBaseAngularMotionTask();
  Task formulateBaseAccelXTask();
  Task formulateActuateTorqueMinTask();
  Task formulateTorqueLimitsTask();
  Task formulateFrictionConeTask();

  vector_t comPosition();
  vector_t comVelocity();
  matrix_t comJacobian();

  matrix_t geometricJacobian(const std::string & frame_name, bool in_base = false);
  matrix_t geometricJacobianTimeVariation(const std::string & frame_name);
  vector3_t getLinearPosition(const std::string & frame_name, bool in_base = false);
  quaternion_t getlinkOrientation(const std::string & frame_name, bool in_base = false);
  vector3_t getSupportPointVelocity(const std::string & jname, bool in_base = false);
  matrix_t getSupportLinearJacobian(const std::string & jname, bool in_base = false);
  matrix_t getSupportLinearJacobianTimeVariation(const std::string & jname);
  quaternion_t generateControlFrameQuaternion();
  
  vector3_t getContactPositionInWheel(const std::string & jname);

  // vector3_t getLinearVelocity(const std::string & frame_name, bool in_base = false);
  // vector3_t getContactPointPosition(const std::string & jname, bool in_base = false);
  // vector3_t getContactPointVelocity(const std::string & jname, bool in_base = false);

  // matrix_t getContactLinearJacobian(const std::string & jname, bool in_base = false);
  // matrix_t getContactLinearJacobianTimeVariation(const std::string & jname);
  // vector3_t getContactPositionRelativeWheel(const std::string & jname, bool in_base = false);
  void updateCommand();

  void addExtraBodyInBase();
  void removeExtraBodyInBase();
  vector_t computeInverseKinematicsLeg(const pinocchio::SE3 & oMdes_l, const pinocchio::SE3 & oMdes_r);
  matrix_t getSMatrix(matrix_t Rcb) // TODO: inverse kinematic 
  {
    Eigen::MatrixXd S(2*Rcb.rows(), 2*Rcb.cols());
    S << -0.5*Rcb, -0.5*Rcb, 0.5*Rcb, -0.5*Rcb;
    return S;
  }

  std::vector<std::string> contact_frame_names_ = {"left_contact", "right_contact"};
  pinocchio::Model * pinModel_, * pinModelFixed_;
  pinocchio::Data * pinData_, * pinDataFixed_;

  matrix3_t Rbw_, Rcw_, Rbc_;

  vector_t q_, qdot_; // with float
  vector_t qd_, qd_dot_; // without float 
  vector3_t vwb, omegawb, pwb;
  quaternion_t qwb;

  vector3_t ground_norm_direction{0, 0, 1};
  matrix_t Jc_, dJc_, Jbase_, dJbase_, Jspl_, dJspl_, Jspr_, dJspr_;

  size_t numDecisionVars_, numContacts_;
  size_t wholeDofNum_, actuatedDofNum_;

  vector_t last_qpSol;
  pinocchio::Inertia base_saved_inertia_;

  std::shared_ptr<RobotControlParameters> param_;
  std::shared_ptr<LowlevelState> low_state_;
  std::shared_ptr<LowlevelCmd> low_command_;
  std::shared_ptr<StateEstimate> state_estimate_;
  std::shared_ptr<DesiredStateCommand> state_command_;
  std::shared_ptr<WheelLeggedData> wheel_legged_data_;  //for estimator
};
// }
#endif
