#include "task/MpcLoco.hpp"

void MpcLoco_Handle::Package_Init(LinearKFPositionVelocityEstimator<float>* pos_est, LegData* legdata, AttitudeData* attidata)
{
    for(int i = 0; i < 4; i++)
    {
        wheel_init_pos_[i] = legdata->q_wheel[i];
        first_stance_[i] = true;
    }
    mpc_test_->initialize(attidata, legdata, pos_est);
    x_vel_cmd_ = 0.;
}

void MpcLoco_Handle::Package_Run(AttitudeData* attitude, LegData* legdata, LinearKFPositionVelocityEstimator<float>* posvelest)
{
    mpc_test_->set_vel_gait(vel_gait_cmd_.x_vel_cmd, vel_gait_cmd_.y_vel_cmd, vel_gait_cmd_.yaw_turn_rate,vel_gait_cmd_.gait_number);
    mpc_test_->run();
    if(mpc_test_->start_solve_mpc)
      sem_post(&mpc_start_);
    if(!vel_gait_cmd_.use_wbc)
    {
      Mat3<float> Kp_stance, Kd_stance;
      Mat3<float> Kp_swing, Kd_swing;
      Kp_stance << 0,0,0,0,0,0,0,0,0;
      Kd_stance << 10,0,0,0,10,0,0,0,10;
      Kp_swing << 700,0,0,0,700,0,0,0,800;
      Kd_swing << 10,0,0,0,10,0,0,0,10;
      Vec3<float> f_cartesian[4];
      Vec3<float> tau[4];

      float x_vel_cmd = vel_gait_cmd_.x_vel_cmd / 0.0948;

      for(int i = 0; i < 4; i++)
      {
          if(posvelest->_stateEstimatorData.result->contactEstimate[i]>0.001) {
              f_cartesian[i] = Kp_stance * (mpc_test_->pFoot_des_leg[i] - legdata->foot_pos_local[i]) + Kd_stance * (mpc_test_->vFoot_des_leg[i] - legdata->foot_vel_local[i]) + mpc_test_->Fr_des_leg[i];
          } else {
              f_cartesian[i] = Kp_swing * (mpc_test_->pFoot_des_leg[i] - legdata->foot_pos_local[i]) + Kd_swing * (mpc_test_->vFoot_des_leg[i] - legdata->foot_vel_local[i]);
          }
          tau[i] = legdata->foot_Jaco_local[i].transpose() * (f_cartesian[i]);
          torque_set_[4*i] = tau[i](0);
          torque_set_[4*i+1] = tau[i](1);
          torque_set_[4*i+2] = tau[i](2);
          if(posvelest->_stateEstimatorData.result->contactEstimate[i]>0.001) {
            if(first_stance_[i]) {
                first_stance_[i] = false;
                wheel_init_abs_pos_[i] = legdata->q_wheel_abs[i];
            }
            wheel_init_abs_pos_[i] += (i < 2 ? x_vel_cmd : -x_vel_cmd) * 0.002;
            torque_set_[4*i+3] = 5 * (wheel_init_abs_pos_[i] - legdata->q_wheel_abs[i]) + 0.5 * ((i < 2 ? x_vel_cmd : -x_vel_cmd) - legdata->qd_wheel[i]);
          } else {
            first_stance_[i] = true;
            torque_set_[4*i+3] = 0.5 * (0 - legdata->qd_wheel[i]);
          }
      }
    } else {
        wbc_data_->pBody_des = mpc_test_->pBody_des;
        wbc_data_->vBody_des = mpc_test_->vBody_des;
        wbc_data_->aBody_des.setZero();

        wbc_data_->pBody_RPY_des = mpc_test_->pBody_RPY_des;
        wbc_data_->vBody_Ori_des.setZero();

        for (size_t leg(0); leg < 4; ++leg)
        {
            wbc_data_->pFoot_des[leg] << mpc_test_->pFoot_des[leg][0], mpc_test_->pFoot_des[leg][1], mpc_test_->pFoot_des[leg][2];
            wbc_data_->vFoot_des[leg] << mpc_test_->vFoot_des[leg][0], mpc_test_->vFoot_des[leg][1], mpc_test_->vFoot_des[leg][2];
            wbc_data_->aFoot_des[leg] = mpc_test_->aFoot_des[leg];
            wbc_data_->Fr_des[leg] = mpc_test_->Fr_des[leg];
        }
        wbc_data_->contact_state = mpc_test_->contact_state;
        wbc_ctrl_->run(wbc_data_, attitude, legdata, posvelest);
        Mat3<float> Kp_, Kd_;
        Kp_ << 4, 0, 0, 0, 4, 0, 0, 0, 4;
        Kd_ << 0.95, 0, 0, 0, 0.2, 0, 0, 0, 0.2;

        x_vel_cmd_ = ApplyVelocityMeetAccelationLimit<float>(x_vel_cmd_, vel_gait_cmd_.x_vel_cmd / 0.0948, -0.56 / 0.0948, 0.56 / 0.0948, -0.4 / 0.0948, 0.4 / 0.0948, 0.002);

        for(int i = 0; i < 4; i++)
        {
        //   printf("wp:%f,%f,%f\n",wbc_ctrl_->joint_pos[i][0],wbc_ctrl_->joint_pos[i][1],wbc_ctrl_->joint_pos[i][2]);
        //   printf("wp:%f,%f,%f\n",wbc_ctrl_->joint_tau[i][0],wbc_ctrl_->joint_tau[i][1],wbc_ctrl_->joint_tau[i][2]);
          Vec3<float> tau = Kp_ * (wbc_ctrl_->joint_pos[i] - Vec3<float>(legdata->q_abad[i], legdata->q_hip[i], legdata->q_knee[i])) + Kd_ * (wbc_ctrl_->joint_vel[i] - Vec3<float>(legdata->qd_abad[i], legdata->qd_hip[i], legdata->qd_knee[i])) + wbc_ctrl_->joint_tau[i];
          torque_set_[4*i] = 1.15 * tau[0];
          torque_set_[4*i+1] = 1.3 * tau[1];
          torque_set_[4*i+2] = 1.3 * tau[2];
          if (posvelest->_stateEstimatorData.result->contactEstimate[i] > 0.001)
          {
              if (first_stance_[i])
              {
                  first_stance_[i] = false;
                  wheel_init_pos_[i] = legdata->q_wheel_abs[i];
              }
              wheel_init_pos_[i] += (i < 2 ? x_vel_cmd_ : -x_vel_cmd_) * 0.002;
              torque_set_[4 * i + 3] = 5 * (wheel_init_pos_[i] - legdata->q_wheel_abs[i]) + 0.5 * ((i < 2 ? x_vel_cmd_ : -x_vel_cmd_) - legdata->qd_wheel[i]);
          }
          else
          {
              first_stance_[i] = true;
              torque_set_[4 * i + 3] = 0.5 * (0 - legdata->qd_wheel[i]);
          }
        }
    }
}