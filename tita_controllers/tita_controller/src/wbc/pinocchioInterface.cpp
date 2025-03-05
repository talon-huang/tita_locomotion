// #include "wbc/pinocchioInterface.h"
// // namespace tita_controller {

// PinocchioInterface::PinocchioInterface(
//   std::shared_ptr<RobotControlParameters> param, std::shared_ptr<LowlevelState> low_state,
//   std::shared_ptr<LowlevelCmd> low_command, std::shared_ptr<StateEstimate> state_estimate,
//   std::shared_ptr<DesiredStateCommand> state_command,
//   std::shared_ptr<WheelLeggedData> wheel_legged_data)
// : param_(param),
//   low_state_(low_state),
//   low_command_(low_command),
//   state_estimate_(state_estimate),
//   state_command_(state_command),
//   wheel_legged_data_(wheel_legged_data)
// {
//   rigid_body_data_ = std::make_shared<RigidBody>();
// }

// void PinocchioInterface::setupPinocchioInterface(const bool & verbose)
// {
//   pinModel_ = new pinocchio::Model();
//   pinModelFixed_ = new pinocchio::Model();

//   try {
//     pinocchio::urdf::buildModel(
//       param_->robot_description, pinocchio::JointModelFreeFlyer(), *pinModel_, verbose);
//     pinocchio::urdf::buildModel(param_->robot_description, *pinModelFixed_, verbose);
//   } catch (const std::exception & e) {
//     std::cerr << "[pinocchio interface] Pinocchio model create err: " << e.what() << std::endl;
//   }

//   // Add two contact frame
//   pinocchio::Frame frame;
//   frame.placement =
//     pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(param_->wheel_radius, 0.0, 0.0));
//   frame.type = pinocchio::FIXED_JOINT;  // 帧的类型（固定）
//   for (auto i = 0; i < param_->wheel_joint_name.size(); i++) {
//     frame.parent = pinModel_->getJointId(param_->wheel_joint_name[i]);  // 关节 ID
//     frame.name = contact_frame_names_[i];                               // 帧的名称
//     pinModel_->addFrame(frame);
//     frame.parent = pinModelFixed_->getJointId(param_->wheel_joint_name[i]);  // 关节 ID
//     pinModelFixed_->addFrame(frame);
//   }

//   pinData_ = new pinocchio::Data(*pinModel_);
//   pinDataFixed_ = new pinocchio::Data(*pinModelFixed_);

//   q_.setZero(pinModel_->nq);
//   qdot_.setZero(pinModel_->nv);

//   wholeDofNum_ = pinModel_->nv;
//   actuatedDofNum_ = wholeDofNum_ - 6;
//   numContacts_ = 2;
//   numDecisionVars_ = wholeDofNum_ + numContacts_ * 2 + actuatedDofNum_;
//   last_qpSol.setZero(numDecisionVars_);

//   base_saved_inertia_ =
//     pinModel_->inertias[pinModel_->frames[pinModel_->getFrameId(param_->base_name)].parent];
// }

// void PinocchioInterface::updateState()
// {
//   addExtraBodyInBase();

//   q_.setZero(pinModel_->nq);
//   qdot_.setZero(pinModel_->nv);
//   omegawb = state_estimate_->omegaBody;
//   qwb = state_estimate_->orientation;
//   pwb = state_estimate_->position;
//   vwb = state_estimate_->vWorld;
//   q_.segment(0, 3) = state_estimate_->position;
//   q_.segment(3, 4) << state_estimate_->orientation[1], state_estimate_->orientation[2],
//     state_estimate_->orientation[3], state_estimate_->orientation[0];
//   qdot_.segment(0, 3) = state_estimate_->vBody;
//   qdot_.segment(3, 3) = state_estimate_->omegaBody;
//   q_.segment(7, pinModel_->nq - 7) = low_state_->q;
//   qdot_.segment(6, pinModel_->nv - 6) = low_state_->dq;

//   // Update contact frame
//   for (auto name : contact_frame_names_) {
//     pinModel_->frames[pinModel_->getFrameId(name)].placement =
//       pinocchio::SE3(Eigen::Matrix3d::Identity(), getContactPositionInWheel(name));
//     pinModelFixed_->frames[pinModelFixed_->getFrameId(name)].placement =
//       pinocchio::SE3(Eigen::Matrix3d::Identity(), getContactPositionInWheel(name));
//   }

//   // pinochhio update
//   pinocchio::forwardKinematics(*pinModel_, *pinData_, q_, qdot_);
//   pinocchio::updateFramePlacements(*pinModel_, *pinData_);
//   pinocchio::computeJointJacobians(*pinModel_, *pinData_, q_);
//   pinocchio::computeJointJacobiansTimeVariation(*pinModel_, *pinData_, q_, qdot_);
//   pinocchio::crba(*pinModel_, *pinData_, q_);
//   pinData_->M.triangularView<Eigen::StrictlyLower>() =
//     pinData_->M.transpose().triangularView<Eigen::StrictlyLower>();
//   pinocchio::nonLinearEffects(*pinModel_, *pinData_, q_, qdot_);
//   // robot kinematic states update
//   scalar_t threshold = 0.001;
//   size_t index = 0;

//   std::vector<vector3_t> omegabc(2);
//   for (auto name : contact_frame_names_) {
//     auto float_frame = pinModel_->getFrameId(name);
//     pinocchio::Data::Matrix6x J_float(6, pinModel_->nv);
//     J_float.setZero();
//     wheel_legged_data_->contact_position_world[index] = pinData_->oMf[float_frame].translation();
//     pinocchio::computeFrameJacobian(
//       *pinModel_, *pinData_, q_, float_frame, pinocchio::LOCAL_WORLD_ALIGNED, J_float);
//     wheel_legged_data_->contact_velocity_world[index] = J_float.topRows(3) * qdot_;
//     wheel_legged_data_->support_velocity_world[index] =
//       getSupportPointVelocity(param_->wheel_joint_name[index], false);

//     auto fixed_frame = pinModelFixed_->getFrameId(name);
//     pinocchio::Data::Matrix6x J_fixed(6, pinModelFixed_->nv);
//     J_fixed.setZero();
//     wheel_legged_data_->contact_position_local[index] =
//       pinDataFixed_->oMf[fixed_frame].translation();
//     pinocchio::computeFrameJacobian(
//       *pinModelFixed_, *pinDataFixed_, q_.tail(pinModelFixed_->nv), fixed_frame,
//       pinocchio::LOCAL_WORLD_ALIGNED, J_fixed);
//     wheel_legged_data_->contact_velocity_local[index] =
//       J_fixed.topRows(3) * qdot_.tail(pinModelFixed_->nv);
//     omegabc[index] = J_fixed.bottomRows(3) * qdot_.tail(pinModelFixed_->nv);
//     wheel_legged_data_->support_velocity_local[index] =
//       getSupportPointVelocity(param_->wheel_joint_name[index], true);

//     matrix_t jt, jtinv;
//     jt = (state_estimate_->rCtrl * geometricJacobian(name).topRows(3).rightCols(pinModel_->nv - 6))
//            .transpose();
//     pseudoInverse(jt, threshold, jtinv);
//     wheel_legged_data_->contact_forces[index] = jtinv * low_state_->tau_est;
//     index++;
//   }

//   Rbw_ = state_estimate_->rBody;
//   Rcw_ = state_estimate_->rCtrl;
//   Rbc_ = Rbw_ * Rcw_.transpose();
//   // update state
//   Eigen::VectorXd x(6);
//   x << wheel_legged_data_->contact_position_local[0], wheel_legged_data_->contact_position_local[1];
//   Eigen::MatrixXd s = getSMatrix(Rbc_.transpose());
//   auto y = s * x;
//   // TODO: 速度没计算明白，先注释掉
//   // Eigen::VectorXd x_dot(6);
//   // x_dot << wheel_legged_data_->contact_velocity_local[0],
//   //   wheel_legged_data_->contact_velocity_local[1];
//   // Eigen::MatrixXd s_dot = Eigen::MatrixXd::Zero(6, 6);
//   // vector3_t omega = (omegabc[0] + omegabc[1])/2.0;
//   // s_dot.block(0, 0, 3, 3) = -0.5 * ori::wedge(-omega) * Rbc_.transpose();
//   // s_dot.block(0, 3, 3, 3) = -0.5 * ori::wedge(-omega) * Rbc_.transpose();

//   // auto y_dot = s * x_dot + s_dot * x;

//   // vector3_t sum_pos =
//   //   -state_estimate_->rCtrl * (0.5 * (wheel_legged_data_->contact_position_world[0] +
//   //                                     wheel_legged_data_->contact_position_world[1]) -
//   //                              pwb);
//   // vector3_t dif_pos =
//   //   state_estimate_->rCtrl * (0.5 * (wheel_legged_data_->contact_position_world[0] -
//   //                                    wheel_legged_data_->contact_position_world[1]));

//   vector3_t sum_vel =
//     -state_estimate_->rCtrl * (0.5 * (wheel_legged_data_->support_velocity_world[0] +
//                                       wheel_legged_data_->support_velocity_world[1]) -
//                                vwb);
//   vector3_t dif_vel =
//     state_estimate_->rCtrl * (0.5 * (wheel_legged_data_->support_velocity_world[0] -
//                                      wheel_legged_data_->support_velocity_world[1]));

//   wheel_legged_data_->pose_position << 0, y(1), y(2);
//   wheel_legged_data_->pose_position_dot << 0, sum_vel(1), sum_vel(2);

//   vector3_t rpy = ori::quatToRPY(qwb);
//   wheel_legged_data_->pose_rpy << rpy(0), rpy(1), y(3);
//   wheel_legged_data_->pose_rpy_dot << omegawb(0), omegawb(1), dif_vel(0);

//   wheel_legged_data_->twist_linear << state_estimate_->rCtrl.row(0) * comVelocity(), 0, 0;
//   wheel_legged_data_->twist_linear_int(0) += wheel_legged_data_->twist_linear(0) * param_->dt;

//   wheel_legged_data_->twist_angular << 0, 0, omegawb(2);
//   wheel_legged_data_->twist_angular_int << 0, 0, rpy(2);

//   wheel_legged_data_->two_wheel_distance = y(4);
//   wheel_legged_data_->two_wheel_distance_dot = -dif_vel(1);  //TODO: 方向问题

//   wheel_legged_data_->com_position_relative =
//     state_estimate_->rCtrl * (comPosition() - (wheel_legged_data_->contact_position_world[0] +
//                                                wheel_legged_data_->contact_position_world[1]) /
//                                                 2.0);
//   wheel_legged_data_->com_velocity_relative =
//     state_estimate_->rCtrl * (comVelocity() - (wheel_legged_data_->support_velocity_world[0] +
//                                                wheel_legged_data_->support_velocity_world[1]) /
//                                                 2.0);
//   removeExtraBodyInBase();
// }

// vector_t PinocchioInterface::computeInverseKinematicsLeg(
//   const pinocchio::SE3 & oMdes_l, const pinocchio::SE3 & oMdes_r)
// {
//   std::vector<pinocchio::SE3> oMdes;
//   oMdes.push_back(oMdes_l);
//   oMdes.push_back(oMdes_r);

//   const double eps = 1e-4;
//   const int IT_MAX = 100;
//   // const double DT = 1e-1;
//   // const double damp = 1e-0;
//   pinocchio::Data data = pinocchio::Data(*pinModelFixed_);

//   pinocchio::Data::Matrix6x J(6, pinModelFixed_->nv);
//   J.setZero();

//   Eigen::Vector4d err;
//   vector_t qd = q_.tail(pinModelFixed_->nv);

//   for (size_t index = 0; index < contact_frame_names_.size(); index++) {
//     bool success = false;
//     auto FRAME_ID = pinModelFixed_->getFrameId(contact_frame_names_[index]);
//     // auto start = std::chrono::high_resolution_clock::now();
//     for (int i = 0;; i++) {
//       pinocchio::forwardKinematics(*pinModelFixed_, data, qd);
//       pinocchio::computeFrameJacobian(
//         *pinModelFixed_, data, qd, FRAME_ID, pinocchio::LOCAL_WORLD_ALIGNED,
//         J);  // J in joint frame
//       const pinocchio::SE3 oMnow = data.oMf[FRAME_ID];
//       auto pos_err = oMdes[index].translation() - oMnow.translation();
//       auto rot_err = pinocchio::log3(
//         oMdes[index].rotation() *
//         oMnow.rotation().transpose());  // pinocchio::log3(iMd.rotation());

//       Eigen::MatrixXd Jtop(4, 4);
//       Jtop.topRows(3) = J.block(0, 4 * index, 3, 4);
//       Jtop.bottomRows(1) = J.block(4, 4 * index, 1, 4);

//       // Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(Jtop.rows(), Jtop.rows());
//       auto Jtop_pi =
//         Jtop.transpose() * (Jtop * Jtop.transpose() /*+ damp * damp * identity*/).inverse();

//       pinocchio::updateFramePlacements(*pinModelFixed_, data);
//       err << pos_err, rot_err(1);
//       qd.segment(4 * index, 4) += Jtop_pi * err;  // WHY?

//       if (err.norm() < eps) {
//         success = true;
//         // std::cout << i << ": error = " << err.transpose() << std::endl;
//         break;
//       }
//       if (i >= IT_MAX) {
//         success = false;
//         // std::cout << i << ": error = " << err.transpose() << std::endl;
//         break;
//       }
//       // if (!(i % 10)) std::cout << i << ": error = " << err.transpose() << std::endl;
//     }
//     // auto end = std::chrono::high_resolution_clock::now();
//     // std::chrono::duration<double> duration = end - start;
//     // if (success) {
//     // pinocchio::forwardKinematics(*pinModelFixed_, *pinDataFixed_, q);
//     // pinocchio::updateFramePlacements(*pinModelFixed_, *pinDataFixed_);
//     // std::cout << "final position: " << pinDataFixed_->oMf[FRAME_ID].translation().transpose()
//     //           << " in " << duration.count() << " second" << std::endl;
//     // }
//   }

//   return qd;
// }

// void PinocchioInterface::addExtraBodyInBase()  // 出过错误
// {
//   pinocchio::Inertia inetia(
//     rigid_body_data_->mass, rigid_body_data_->com, rigid_body_data_->inertia);
//   pinocchio::JointIndex base_joint_id =
//     pinModel_->frames[pinModel_->getFrameId(param_->base_name)].parent;
//   pinModel_->appendBodyToJoint(base_joint_id, inetia, rigid_body_data_->transform);
// }

// void PinocchioInterface::removeExtraBodyInBase()
// {
//   pinocchio::JointIndex base_joint_id =
//     pinModel_->frames[pinModel_->getFrameId(param_->base_name)].parent;
//   pinModel_->inertias[base_joint_id] = base_saved_inertia_;
// }
// // xyz rx ry rz
// matrix_t PinocchioInterface::geometricJacobian(const std::string & frame_name, bool in_base)
// {
//   pinocchio::Data::Matrix6x J(6, pinModel_->nv);
//   J.fill(0);
//   // Jacobian in pinocchio::WORLD (link) frame
//   pinocchio::Model::FrameIndex link_number = pinModel_->getFrameId(frame_name);
//   pinocchio::getFrameJacobian(
//     *pinModel_, *pinData_, link_number, pinocchio::LOCAL_WORLD_ALIGNED, J);
//   if (in_base) {
//     J.block(0, 0, 6, 6) = matrix_t::Zero(6, 6);
//     J.topRows(3) = ori::quaternionToRotationMatrix(qwb) * J.topRows(3);
//     J.bottomRows(3) = ori::quaternionToRotationMatrix(qwb) * J.bottomRows(3);
//   }
//   return J;
// }

// matrix_t PinocchioInterface::geometricJacobianTimeVariation(const std::string & frame_name)
// {
//   pinocchio::Data::Matrix6x dJ(6, pinModel_->nv);
//   dJ.fill(0);
//   // Jacobian in pinocchio::WORLD (link) frame
//   pinocchio::Model::FrameIndex link_number = pinModel_->getFrameId(frame_name);
//   pinocchio::getFrameJacobianTimeVariation(
//     *pinModel_, *pinData_, link_number, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
//   return dJ;
// }

// vector3_t PinocchioInterface::getLinearPosition(const std::string & frame_name, bool in_base)
// {
//   pinocchio::Model::FrameIndex link_number = pinModel_->getFrameId(frame_name);
//   vector3_t position = pinData_->oMf[link_number].translation();
//   if (in_base) {
//     return ori::quaternionToRotationMatrix(qwb) * (position - pwb);
//   }
//   return position;
// }

// quaternion_t PinocchioInterface::getlinkOrientation(const std::string & frame_name, bool in_base)
// {
//   pinocchio::Model::FrameIndex link_number = pinModel_->getFrameId(frame_name);
//   auto rotation_matrix = pinData_->oMf[link_number].rotation();
//   // Eigen::Quaterniond tempQ = Eigen::Quaterniond(pinData_->oMf[link_number].rotation());
//   if (in_base) {
//     rotation_matrix = ori::quaternionToRotationMatrix(qwb) * rotation_matrix;
//   }
//   return ori::rotationMatrixToQuaternion(rotation_matrix);
// }

// vector_t PinocchioInterface::comPosition()
// {
//   pinocchio::centerOfMass(*pinModel_, *pinData_, q_);
//   return pinData_->com[0];
// }

// matrix_t PinocchioInterface::comJacobian()
// {
//   pinocchio::jacobianCenterOfMass(*pinModel_, *pinData_, q_);
//   return pinData_->Jcom;
// }

// vector_t PinocchioInterface::comVelocity() { return comJacobian() * qdot_; }

// vector3_t PinocchioInterface::getSupportPointVelocity(const std::string & jname, bool in_base)
// {
//   return getSupportLinearJacobian(jname, in_base) * qdot_;
// }
// // reference to world
// quaternion_t PinocchioInterface::generateControlFrameQuaternion()
// {
//   vector3_t dif =
//     wheel_legged_data_->contact_position_world[0] - wheel_legged_data_->contact_position_world[1];
//   dif(2) = 0;
//   dif = dif.normalized();
//   matrix3_t m;
//   m.col(1) = dif;
//   m.col(2) = vector3_t(0, 0, 1);
//   m.col(0) = m.col(1).cross(m.col(2));
//   return ori::rotationMatrixToQuaternion(m.transpose());
// }
// // vsp = w * r
// matrix_t PinocchioInterface::getSupportLinearJacobian(const std::string & jname, bool in_base)
// {
//   vector3_t riw =
//     ori::quaternionToRotationMatrix(getlinkOrientation(jname)) * getContactPositionInWheel(jname);
//   matrix_t Jsp = ori::vectorToSkewMat(riw) * geometricJacobian(jname, in_base).bottomRows(3);
//   return Jsp;
// }

// matrix_t PinocchioInterface::getSupportLinearJacobianTimeVariation(const std::string & jname)
// {
//   vector3_t riw =
//     ori::quaternionToRotationMatrix(getlinkOrientation(jname)) * getContactPositionInWheel(jname);
//   matrix_t dJsp = ori::vectorToSkewMat(riw) * geometricJacobianTimeVariation(jname).bottomRows(3);
//   return dJsp;
// }

// vector3_t PinocchioInterface::getContactPositionInWheel(const std::string & jname)
// {
//   /* wheel x-axis and ground normal angle
//   WARN: wheel coordinate mast rotation in z-axis */
//   matrix_t Riw = ori::quaternionToRotationMatrix(getlinkOrientation(jname));  // * map_to_y_axis;
//   matrix_t Rwi = Riw.transpose();
//   scalar_t countor_parameter =
//     atan2(-Rwi.row(1) * ground_norm_direction, Rwi.row(0) * ground_norm_direction);
//   vector3_t rww = {
//     -param_->wheel_radius * cos(countor_parameter), param_->wheel_radius * sin(countor_parameter),
//     0};

//   return rww;
// }
// // }

// /*
// q = [IrIB, qwb, q_1 ... q_nj]
// u = [IvIB, IomegaIb, dot_q_1 ... dot_q_nj]
// dot_u = [IaIB, Idot_omegaIb, ddot_q_1 ... ddot_q_nj]
// tau = [tau_1 ... tau_nj]

// decision variable:
// Fc = [Fxl Fzl Fxr Fzr]';
// x = [ddot_q' Fc' tau']';

// Dynamic Formula:
// M * dot_u + C = S' * tau + (Ja + Cf' * Jf)' * Fc;

// */

// void PinocchioInterface::updateWBC()
// {
//   Jc_.resize(numContacts_ * 3, wholeDofNum_);
//   dJc_.resize(numContacts_ * 3, wholeDofNum_);
//   for (size_t i = 0; i < numContacts_; i++) {
//     Jc_.block(3 * i, 0, 3, wholeDofNum_) = geometricJacobian(contact_frame_names_[i]).topRows(3);
//     dJc_.block(3 * i, 0, 3, wholeDofNum_) =
//       geometricJacobianTimeVariation(contact_frame_names_[i]).topRows(3);
//   }

//   Jbase_ = geometricJacobian(param_->base_name);
//   dJbase_ = geometricJacobianTimeVariation(param_->base_name);

//   Jspl_ = getSupportLinearJacobian(param_->wheel_joint_name[0]);
//   Jspr_ = getSupportLinearJacobian(param_->wheel_joint_name[1]);
//   dJspl_ = getSupportLinearJacobianTimeVariation(param_->wheel_joint_name[0]);
//   dJspr_ = getSupportLinearJacobianTimeVariation(param_->wheel_joint_name[1]);

//   // updateHierarchicalWbc();
//   updateWeightedWbc();

//   updateCommand();
// }

// void PinocchioInterface::updateHierarchicalWbc()
// {
//   Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() +
//                formulateFrictionConeTask() + formulateNoContactMotionTask();
//   Task task1 = formulateLinearTask() + formulateBaseAccelXTask() + formulateBaseAngularMotionTask();

//   Task task2 = formulateActuateTorqueMinTask() * 0.01;
//   HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
//   last_qpSol = hoQp.getSolutions();
//   low_command_->tau_cmd = last_qpSol.tail(actuatedDofNum_);

//   vector_t contact_forces = last_qpSol.segment(wholeDofNum_, 3 * numContacts_);
//   vector_t base_accel = last_qpSol.head(wholeDofNum_);
// }

// void PinocchioInterface::updateWeightedWbc()  // not useful why?
// {
//   Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() +
//                /*formulateFrictionConeTask() + */
//                formulateNoContactMotionTask();  // TODO: remove friction cone?
//   Task task1 = formulateLinearTask() + formulateBaseAccelXTask() + formulateBaseAngularMotionTask();
//   Task task2 = formulateActuateTorqueMinTask() * 0.01;

//   // Constraints
//   Task constraints = task0;
//   size_t numConstraints = constraints.b_.size() + constraints.f_.size();
//   Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
//     numConstraints, numDecisionVars_);
//   vector_t lbA(numConstraints), ubA(numConstraints);
//   A << constraints.a_, constraints.d_;
//   lbA << constraints.b_, -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
//   ubA << constraints.b_, constraints.f_;
//   // Cost
//   Task weighedTask = task1 + task2;
//   Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
//     weighedTask.a_.transpose() * weighedTask.a_;
//   vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
//   // Solve
//   auto qpProblem = qpOASES::QProblem(numDecisionVars_, numConstraints);
//   qpOASES::Options options;
//   options.setToMPC();
//   options.printLevel = qpOASES::PL_NONE;
//   options.enableEqualities = qpOASES::BT_TRUE;
//   qpProblem.setOptions(options);
//   int nWsr = 100;

//   qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
//   vector_t qpSol(numDecisionVars_);

//   qpProblem.getPrimalSolution(qpSol.data());

//   if (!qpProblem.isSolved()) {
//     std::cerr << "ERROR: WeightWBC Not Solved!!!" << std::endl;
//     if (last_qpSol.size() > 0) {
//       qpSol = last_qpSol;
//     }
//   }
//   last_qpSol = qpSol;

//   low_command_->tau_cmd = last_qpSol.tail(actuatedDofNum_);
//   vector_t contact_forces = last_qpSol.segment(wholeDofNum_, 6);
//   vector_t base_accel = last_qpSol.head(wholeDofNum_);
//   // PRINT_MAT(last_qpSol.transpose());
// }

// Task PinocchioInterface::formulateFloatingBaseEomTask()
// {
//   matrix_t A = matrix_t::Zero(wholeDofNum_, numDecisionVars_);
//   vector_t b = vector_t::Zero(wholeDofNum_);
//   matrix_t Jf(numContacts_, wholeDofNum_), Ja(numContacts_ * 2, wholeDofNum_);
//   matrix_t Cf(numContacts_, 2 * numContacts_);
//   Cf.setZero();
//   matrix_t Jc_map;
//   for (size_t i = 0; i < numContacts_; i++) {
//     Jc_map = state_estimate_->rCtrl * Jc_.block(i * 3, 0, 3, wholeDofNum_);
//     Jf.row(i) = Jc_map.row(1);
//     Ja.row(i * 2) = Jc_map.row(0);
//     Ja.row(i * 2 + 1) = Jc_map.row(2);
//     Cf(i, i * 2 + 1) = -param_->sliding_friction_coefficient * tanh(Jf.row(i) * qdot_);
//   }
//   matrix_t S;  // select actuate joint
//   S.setZero(actuatedDofNum_, wholeDofNum_);
//   S.block(0, 6, actuatedDofNum_, actuatedDofNum_) =
//     matrix_t::Identity(actuatedDofNum_, actuatedDofNum_);
//   A << pinData_->M, -Ja.transpose() - Jf.transpose() * Cf, -S.transpose();
//   b = -pinData_->nle;
//   return {A, b, matrix_t(0, numDecisionVars_), vector_t()};
// }

// Task PinocchioInterface::formulateNoContactMotionTask()
// {
//   matrix_t A = matrix_t::Zero(numContacts_ * 2, numDecisionVars_);
//   vector_t b = vector_t::Zero(numContacts_ * 2);
//   matrix_t Ja(2 * numContacts_, wholeDofNum_), dJa(2 * numContacts_, wholeDofNum_);
//   matrix_t Jc_map, dJc_map;
//   for (size_t i = 0; i < numContacts_; i++) {
//     Jc_map = state_estimate_->rCtrl * Jc_.block(i * 3, 0, 3, wholeDofNum_);
//     dJc_map = state_estimate_->rCtrl * dJc_.block(i * 3, 0, 3, wholeDofNum_);
//     Ja.row(i * 2) = Jc_map.row(0);
//     Ja.row(i * 2 + 1) = Jc_map.row(2);
//     dJa.row(i * 2) = dJc_map.row(0);
//     dJa.row(i * 2 + 1) = dJc_map.row(2);
//   }

//   A.block(0, 0, numContacts_ * 2, wholeDofNum_) = Ja;
//   b = -dJa * qdot_;
//   return {A, b, matrix_t(0, numDecisionVars_), vector_t()};
// }

// void PinocchioInterface::updateCommand()  // use inverse kinematic
// {
//   vector_t yd(6), yd_dot(6);
//   yd << state_command_->desire_data_->pose_position,
//     state_command_->desire_data_->pose_rpy(rpy::YAW),
//     state_command_->desire_data_->two_wheel_distance, 0.0;
//   yd_dot << state_command_->desire_data_->pose_position_dot,
//     state_command_->desire_data_->pose_rpy_dot(rpy::YAW),
//     state_command_->desire_data_->two_wheel_distance_dot, 0.0;

//   vector3_t eular_d;
//   eular_d << state_command_->desire_data_->pose_rpy(rpy::ROLL),
//     state_command_->desire_data_->pose_rpy(rpy::PITCH), 0.0;
//   matrix_t Rcb_d = ori::rpyToRotMat(eular_d).transpose();

//   matrix_t sd = getSMatrix(Rcb_d);
//   // matrix_t::Zero(6, 6);
//   // sd.block(0, 0, 3, 3) = -0.5 * Rcb_d;
//   // sd.block(0, 3, 3, 3) = -0.5 * Rcb_d;
//   // sd.block(3, 0, 3, 3) = 0.5 * matrix3_t::Identity();
//   // sd.block(3, 3, 3, 3) = -0.5 * matrix3_t::Identity();

//   vector_t xd = sd.inverse() * yd;

//   pinocchio::updateFramePlacements(*pinModelFixed_, *pinDataFixed_);
//   std::vector<Eigen::Matrix3d> contact_rotation_body;
//   contact_rotation_body.resize(2);

//   for (size_t i = 0; i < contact_frame_names_.size(); i++) {
//     auto FRAME_ID = pinModelFixed_->getFrameId(contact_frame_names_[i]);
//     contact_rotation_body[i] = pinDataFixed_->oMf[FRAME_ID].rotation();
//   }
//   const pinocchio::SE3 oMdes_l(contact_rotation_body[0], vector3_t(xd.segment<3>(0))),
//     oMdes_r(contact_rotation_body[1], vector3_t(xd.segment<3>(3)));

//   // inverse kinematic
//   qd_ = computeInverseKinematicsLeg(oMdes_l, oMdes_r);
//   qd_dot_ = vector_t::Zero(pinModelFixed_->nv);
//   low_command_->qd = qd_;
//   low_command_->qd_dot = qd_dot_;
//   // low_command_->kd.setConstant(2.0);
//   // std::cout << " Rcb_d \n " << Rcb_d << "\n " << std::endl;
//   // std::cout << std::fixed << std::setprecision(6);

//   // std::cout << "yd: " << yd.transpose() << std::endl;
//   // std::cout << "xd: " << xd.transpose() << std::endl;
//   // std::cout << "q : " << q_.tail(pinModelFixed_->nv).transpose() << std::endl;
//   // std::cout << "qd: " << qd_.transpose() << std::endl;
//   // std::cout << "err: " << q_.tail(pinModelFixed_->nv).transpose() - qd_.transpose() << std::endl;
//   // std::cout << "----------------------------------------------------------" << std::endl;
//   auto err = q_.tail(pinModelFixed_->nv) - qd_;
// }
// Task PinocchioInterface::formulateLinearTask()
// {
//   size_t task_size = 6;

//   matrix_t sum_Jw =
//     state_estimate_->rCtrl * (Jbase_.topRows(3) - 0.5 * (Jspl_ + Jspr_));  //(Jw_l + Jw_r) / 2.0f;
//   matrix_t dif_Jw = state_estimate_->rCtrl * (Jspl_ - Jspr_);              //(Jw_l - Jw_r) / 2.0f;

//   matrix_t sum_dJw = state_estimate_->rCtrl *
//                      (dJbase_.topRows(3) - 0.5 * (dJspl_ + dJspr_));  //(dJw_l + dJw_r) / 2.0f;
//   matrix_t dif_dJw = state_estimate_->rCtrl * (dJspl_ - dJspr_);      //(dJw_l - dJw_r) / 2.0f;

//   vector_t linear_pos_fd(task_size), linear_vel_fd(task_size), linear_pos_ref(task_size),
//     linear_vel_ref(task_size), linear_accl_ref(task_size);
//   vector_t linear_kp(task_size), linear_kd(task_size);
//   linear_kp << 0.0f, -param_->sum_y_kp, param_->sum_z_kp, param_->dif_x_kp, -param_->dif_y_kp, 0.0f;
//   linear_kd << 0.0f, -param_->sum_y_kd, param_->sum_z_kd, param_->dif_x_kd, -param_->dif_y_kd, 0.0f;

//   linear_pos_fd << 0.0, wheel_legged_data_->pose_position(point::Y),
//     wheel_legged_data_->pose_position(point::Z), wheel_legged_data_->pose_rpy(rpy::YAW),
//     wheel_legged_data_->two_wheel_distance, 0.0;

//   linear_vel_fd << 0.0, wheel_legged_data_->pose_position_dot(point::Y),
//     wheel_legged_data_->pose_position_dot(point::Z), wheel_legged_data_->pose_rpy_dot(rpy::YAW),
//     wheel_legged_data_->two_wheel_distance_dot, 0.0;

//   linear_pos_ref << 0.0, state_command_->desire_data_->pose_position(point::Y),
//     state_command_->desire_data_->pose_position(point::Z),
//     state_command_->desire_data_->pose_rpy(rpy::YAW),
//     state_command_->desire_data_->two_wheel_distance, 0.0;

//   linear_vel_ref << 0.0, state_command_->desire_data_->pose_position_dot(point::Y),
//     state_command_->desire_data_->pose_position_dot(point::Z),
//     state_command_->desire_data_->pose_rpy_dot(rpy::YAW),
//     state_command_->desire_data_->two_wheel_distance_dot, 0.0;

//   vector_t linear_accl_des = linear_kp.cwiseProduct(linear_pos_ref - linear_pos_fd) +
//                              linear_kd.cwiseProduct(linear_vel_ref - linear_vel_fd);
//   matrix_t A = matrix_t::Zero(task_size, numDecisionVars_);
//   vector_t b = vector_t::Zero(task_size);
//   A.block(0, 0, 3, wholeDofNum_) = sum_Jw;
//   A.block(3, 0, 3, wholeDofNum_) = dif_Jw;

//   b.segment(0, 3) = linear_accl_des.segment(0, 3) - sum_dJw * qdot_;
//   b.segment(3, 3) = linear_accl_des.segment(3, 3) - dif_dJw * qdot_;

//   matrix_t A_cut = matrix_t::Zero(4, numDecisionVars_);
//   vector_t b_cut = vector_t::Zero(4);
//   if (actuatedDofNum_ == 8) {
//     A_cut << A.block(1, 0, 4, numDecisionVars_);
//     b_cut << b.segment(1, 4);
//   } else {
//     A_cut.resize(2, numDecisionVars_);
//     b_cut.resize(2);
//     A_cut << A.block(2, 0, 2, numDecisionVars_);
//     b_cut << b.segment(2, 2);
//   }
//   return {A_cut, b_cut, matrix_t(0, numDecisionVars_), vector_t()};
// }

// Task PinocchioInterface::formulateBaseAngularMotionTask()
// {
//   // ref
//   Quat<scalar_t> ori_cmd(1, 0, 0, 0);
//   vector3_t omegawb_ref(0, 0, 0), omegawb_dot_ref(0, 0, 0);
//   vector_t angular_kp(3), angular_kd(3);
//   vector3_t eul_ref(0, 0, 0);
//   angular_kp << param_->roll_kp, param_->pitch_kp, param_->yaw_kp;
//   angular_kd << param_->roll_kd, param_->pitch_kd, param_->yaw_kd;

//   eul_ref << state_command_->desire_data_->pose_rpy(rpy::ROLL),
//     state_command_->desire_data_->pose_rpy(rpy::PITCH),
//     state_command_->desire_data_->twist_angular_int(point::Z);
//   omegawb_ref << state_command_->desire_data_->pose_rpy_dot(rpy::ROLL),
//     state_command_->desire_data_->pose_rpy_dot(rpy::PITCH),
//     state_command_->desire_data_->twist_angular(point::Z);

//   ori_cmd = ori::rpyToQuat(eul_ref);
//   Quat<scalar_t> link_ori_inv;
//   link_ori_inv[0] = qwb[0];
//   link_ori_inv[1] = -qwb[1];
//   link_ori_inv[2] = -qwb[2];
//   link_ori_inv[3] = -qwb[3];

//   Quat<scalar_t> ori_err = ori::quatProduct(ori_cmd, link_ori_inv);

//   if (ori_err[0] < 0.) {
//     ori_err *= (-1.);
//   }
//   vector3_t ori_err_so3;
//   ori::quaternionToso3(ori_err, ori_err_so3);
//   //
//   vector_t angular_accl_des =
//     angular_kp.cwiseProduct(ori_err_so3) +
//     angular_kd.cwiseProduct(
//       ori::quaternionToRotationMatrix(qwb).transpose() * (omegawb_ref - omegawb)) +
//     omegawb_dot_ref;
//   // a b
//   matrix_t A = matrix_t::Zero(3, numDecisionVars_);
//   vector_t b = vector_t::Zero(3);
//   // 1. 底座姿态任务 3dof
//   A.block(0, 0, 3, wholeDofNum_) = Jbase_.bottomRows(3);
//   b.segment(0, 3) = angular_accl_des - dJbase_.bottomRows(3) * qdot_;

//   return {A, b, matrix_t(0, numDecisionVars_), vector_t()};
// }
// // #def
// /*
// // centroidal dynamics:
//   - m*ddp_com = f_c- m*g;
//   - dotL = tau_c + (r - p_com) x f_c;
// */
// Task PinocchioInterface::formulateBaseAccelXTask()
// {
//   size_t lqr_states_size /*, lqr_inputs_size*/;
//   lqr_states_size = 4 /*, lqr_inputs_size = 1*/;
//   vector_t x_qr_fd(lqr_states_size), x_qr_ref(lqr_states_size);

//   x_qr_fd << wheel_legged_data_->twist_linear_int(point::X),
//     wheel_legged_data_->twist_linear(point::X), wheel_legged_data_->com_position_relative(point::X),
//     wheel_legged_data_->com_velocity_relative(point::X);

//   x_qr_ref << state_command_->desire_data_->twist_linear_int(point::X),
//     state_command_->desire_data_->twist_linear(point::X), 0, 0;

//   // a b
//   vector_t com_accl_des = wheel_legged_data_->K_ * (x_qr_ref - x_qr_fd);
//   // a b
//   matrix_t A = matrix_t::Zero(1, numDecisionVars_);
//   vector_t b = vector_t::Zero(1);

//   matrix_t JCcom = state_estimate_->rCtrl * (comJacobian() - 0.5 * (Jspl_ + Jspr_));
//   // matrix_t dJCcom;
//   // dJCcom.setZero(3, wholeDofNum_);  // TODO: Djcom没有计算 为0
//   A.block(0, 0, 1, wholeDofNum_) = JCcom.row(0);
//   b.segment(0, 1) = com_accl_des;  // - dJCcom.row(1) * qdot_;

//   return {A, b, matrix_t(0, numDecisionVars_), vector_t()};
// }

// Task PinocchioInterface::formulateActuateTorqueMinTask()
// {
//   matrix_t A = matrix_t::Zero(actuatedDofNum_, numDecisionVars_);
//   vector_t b = vector_t::Zero(actuatedDofNum_);
//   A.block(0, numDecisionVars_ - actuatedDofNum_, actuatedDofNum_, actuatedDofNum_) =
//     matrix_t::Identity(actuatedDofNum_, actuatedDofNum_);
//   return {A, b, matrix_t(0, numDecisionVars_), vector_t()};
// }
// // unequal
// Task PinocchioInterface::formulateTorqueLimitsTask()
// {
//   matrix_t C = matrix_t::Zero(2 * actuatedDofNum_, numDecisionVars_);
//   vector_t d = vector_t::Zero(2 * actuatedDofNum_);
//   vector_t torqueLimits = Eigen::Map<Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>>(
//     param_->torque_limit.data(), param_->torque_limit.size());
//   d << torqueLimits, torqueLimits;
//   matrix_t i = matrix_t::Identity(actuatedDofNum_, actuatedDofNum_);
//   C.block(0, numDecisionVars_ - actuatedDofNum_, actuatedDofNum_, actuatedDofNum_) = i;
//   C.block(actuatedDofNum_, numDecisionVars_ - actuatedDofNum_, actuatedDofNum_, actuatedDofNum_) =
//     -i;
//   return {matrix_t(0, numDecisionVars_), vector_t(), C, d};
// }

// Task PinocchioInterface::formulateFrictionConeTask()
// {
//   matrix_t C = matrix_t::Zero(3 * numContacts_, numDecisionVars_);
//   vector_t d = vector_t::Zero(3 * numContacts_);
//   matrix_t frictionPyramic(3, 2);  // clang-format off
//   frictionPyramic << 0, -1,
//                      1, -param_->static_friction_coefficient,
//                     -1, -param_->static_friction_coefficient;  // clang-format on
//   for (size_t i = 0; i < numContacts_; i++) {
//     C.block(3 * i, wholeDofNum_ + 2 * i, 3, 2) = frictionPyramic;
//   }
//   return {matrix_t(0, numDecisionVars_), vector_t(), C, d};
// }
// // vector3_t PinocchioInterface::getContactPositionRelativeWheel(
// //   const std::string & jname, bool in_base)
// // {
// //   matrix_t Riw = ori::quaternionToRotationMatrix(getlinkOrientation(jname));  // * map_to_y_axis;
// //   matrix_t Rwi = Riw.transpose();
// //   /* wheel x-axis and ground normal angle
// //   WARN: wheel coordinate mast rotation in z-axis */
// //   scalar_t countor_parameter =
// //     atan2(-Rwi.row(1) * ground_norm_direction, Rwi.row(0) * ground_norm_direction);
// //   vector3_t rww = {
// //     -param_->wheel_radius * cos(countor_parameter), param_->wheel_radius * sin(countor_parameter),
// //     0};
// //   if (in_base)
// //     return ori::quaternionToRotationMatrix(qwb) * Riw * rww;
// //   else
// //     return Riw * rww;
// // }
// // vector3_t PinocchioInterface::getLinearVelocity(const std::string & frame_name, bool in_base)
// // {
// //   matrix_t J = geometricJacobian(frame_name, in_base).topRows(3);
// //   return J * qdot_;
// // }

// // vector3_t PinocchioInterface::getContactPointPosition(const std::string & jname, bool in_base)
// // {
// //   return getLinearPosition(jname, in_base) + getContactPositionRelativeWheel(jname, in_base);
// // }

// // vector3_t PinocchioInterface::getContactPointVelocity(const std::string & jname, bool in_base)
// // {
// //   return getContactLinearJacobian(jname, in_base) * qdot_;
// // }

// // matrix_t PinocchioInterface::getContactLinearJacobian(const std::string & jname, bool in_base)
// // {
// //   matrix_t Jc = geometricJacobian(jname, in_base).topRows(3) -
// //                 ori::wedge(getContactPositionRelativeWheel(jname)) *
// //                   geometricJacobian(jname, in_base).bottomRows(3);
// //   return Jc;
// // }

// // matrix_t PinocchioInterface::getContactLinearJacobianTimeVariation(const std::string & jname)
// // {
// //   matrix_t dJc = geometricJacobianTimeVariation(jname).topRows(3) -
// //                  ori::wedge(getContactPositionRelativeWheel(jname)) *
// //                    geometricJacobianTimeVariation(jname).bottomRows(3) -
// //                  ori::wedge(geometricJacobian(jname).bottomRows(3) * qdot_) *
// //                    ori::wedge(getContactPositionRelativeWheel(jname)) *
// //                    geometricJacobian(jname).bottomRows(3);
// //   return dJc;
// // }
