/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "incestimator/PositionVelocityEstimator.h"

/*!
 * Initialize the state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {
  T dt = 0.002;
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = T(1);
  _C(26, 14) = T(1);
  _C(25, 11) = T(1);
  _C(24, 8) = T(1);
  _P.setIdentity();
  _P = T(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  _R0.setIdentity();
  contact_schedule << 0.5, 0.5, 0.5, 0.5;
  // qd_wheel_tmp.setZero();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
// foot_height_sensor_noise      :  0.001
// foot_process_noise_position   :  0.002
// foot_sensor_noise_position    :  0.001
// foot_sensor_noise_velocity    :  0.1
// imu_process_noise_position    :  0.02
// imu_process_noise_velocity    :  0.02
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run() {
//   T process_noise_pimu =
//       this->_stateEstimatorData.parameters->imu_process_noise_position;
//   T process_noise_vimu =
//       this->_stateEstimatorData.parameters->imu_process_noise_velocity;
//   T process_noise_pfoot =
//       this->_stateEstimatorData.parameters->foot_process_noise_position;
//   T sensor_noise_pimu_rel_foot =
//       this->_stateEstimatorData.parameters->foot_sensor_noise_position;
//   T sensor_noise_vimu_rel_foot =
//       this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
//   T sensor_noise_zfoot =
//       this->_stateEstimatorData.parameters->foot_height_sensor_noise;
  T process_noise_pimu = 0.02;
  T process_noise_vimu = 0.02;
  T process_noise_pfoot = 0.002;
  T sensor_noise_pimu_rel_foot = 0.001;
  T sensor_noise_vimu_rel_foot = 0.1;
  T sensor_noise_zfoot = 0.001;

  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<T> g(0, 0, T(-9.81));
  Mat3<T> Rbod = attidata->rot_body.transpose();//this->_stateEstimatorData.result->rBody.transpose();
  // in old code, Rbod * se_acc + g
//   Vec3<T> a = this->_stateEstimatorData.result->aWorld + g;
  Vec3<T> a = attidata->acc_world + g;
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();
  Vec3<T> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  // int contact_num = 0;
  // float v_wheel = 0.0;
  // for(int i = 0; i < 4; i++) {
  //   if(contact_schedule[i] > 0.1 && contact_schedule[i] < 0.9) {
  //     contact_num++;
  //     v_wheel += (i < 2 ? 1.0 : -1.0) * legdata->qd_wheel[i] * 0.0948;
  //   }
  // }
  // float v_wheel_to_world = 0.0;
  // static float v_wheel_to_world_last = 0.0;
  // if(contact_num != 0) {
  //   v_wheel_to_world = v_wheel / contact_num;
  // }
  // v0[0] += (v_wheel_to_world - v_wheel_to_world_last) * cos(attidata->rpy[2]);
  // v0[1] += (v_wheel_to_world - v_wheel_to_world_last) * sin(attidata->rpy[2]);
  // v_wheel_to_world_last = v_wheel_to_world;

  // static int debug_count = 0;
  // if(debug_count++ % 100 == 0) {
  //   printf("v_com:%f, %f\n", v0[0], v0[1]);
  //   printf("v_wheel:%f, %f\n", v_wheel_to_world * cos(attidata->rpy[2]), v_wheel_to_world * sin(attidata->rpy[2]));
  // }

  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
    // Quadruped<T>& quadruped =
    //     *(this->_stateEstimatorData.legControllerData->quadruped);
    // Vec3<T> ph = quadruped.getHipLocation(i);  // hip positions relative to CoM
    
    Vec3<T> ph = legdata->GetHipLocation(i);  // hip positions relative to CoM
    // hw_i->leg_controller->leg_datas[i].p; 
    // Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
    Vec3<T> p_rel = ph + legdata->foot_pos_local[i];
    // hw_i->leg_controller->leg_datas[i].v;
    // Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v;
    Vec3<T> dp_rel = legdata->foot_vel_local[i];
    Vec3<T> p_f = Rbod * p_rel;
    // Vec3<T> dp_f =
    //     Rbod *
    //     (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);
    Vec3<T> dp_f = attidata->omega_world.cross(p_rel) +
        Rbod *
        (dp_rel);
    // qd_wheel_tmp[i] = 0.8 * legdata->qd_wheel[i] + 0.2 * qd_wheel_tmp[i];
    // Vec3<T> v_wheel_f = Rbod * Vec3<T>((i < 2 ? 1.0 : -1.0) * qd_wheel_tmp[i] * 0.0948, 0.0, 0.0);
    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    T trust = T(1);
    // T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));
    T phase = fmin(contact_schedule(i), T(1));
    //T trust_window = T(0.25);
    T trust_window = T(0.2);

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }
    //T high_suspect_number(1000);
    T high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    // _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f  + 0.*v_wheel_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<T, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, 28, 1> ey = y - yModel;
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

//   todo compute LU only once
  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }

  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody = attidata->rot_body * this->_stateEstimatorData.result->vWorld;
  this->_stateEstimatorData.result->contactEstimate = contact_schedule;
}

template class LinearKFPositionVelocityEstimator<float>;
// template class LinearKFPositionVelocityEstimator<double>;

