/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-01-04 14:31:23
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-04-22 14:05:17
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/src/task/WbcStand.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "task/WbcStand.hpp"
#include "common/Utilities.h"

void WbcStand_Handle::Package_Init(LinearKFPositionVelocityEstimator<float>* pos_est, LegData* legdata, AttitudeData* attidata)
{
    ini_body_pos_ = pos_est->_stateEstimatorData.result->position;
    if(ini_body_pos_[2] < 0.2) {
        ini_body_pos_[2] = 0.25;
    }
    ini_body_ori_rpy_ = attidata->rpy;
    body_weight_         = 30 * 9.81;

    for(size_t i( 0 ); i < 4; ++i) {
        wbc_data_->Fr_des[i].setZero();
        wbc_data_->Fr_des[i][2] = body_weight_ / 4.;
    }
    for(int i = 0; i < 4; i++) {
        wheel_init_pos_abs_[i] = legdata->q_wheel_abs[i];
    }
    wbc_ctrl_->setFloatingBaseWeight(1000);
    pos_est->set_contact(Vec4<float>(0.5,0.5,0.5,0.5));

    x_vel_cmd_ = 0.;
    x_vel_des_ = 0.;
    pitch_cmd_ = 0.;
    pitch_compensate_ = 0.;
}

void WbcStand_Handle::Package_Run(AttitudeData* attitude, LegData* legdata, LinearKFPositionVelocityEstimator<float>* posvelest)
{
    Vec3<float> tmp(0.,0.,0.);
    for(int i = 0; i < 4; i++)
      tmp += attitude->rot_body.transpose() * (legdata->GetHipLocation(i) + legdata->foot_pos_local[i]);
    wbc_data_->pBody_des = posvelest->_stateEstimatorData.result->position + Vec3<float>(tmp[0]/4.0,tmp[1]/4,0.);
    // float yaw = posvelest->_stateEstimatorData.result->rpy[2];
    // wbc_data_->pBody_des[0] += 1.5 * 0.0948 * cos(yaw) * 0.002;
    // wbc_data_->pBody_des[1] += 1.5 * 0.0948 * sin(yaw) * 0.002;
    wbc_data_->pBody_des[2] = 0.25;
    wbc_data_->vBody_des.setZero();
    // float yaw = posvelest->_stateEstimatorData.result->rpy[2];
    // float vel = 0.0948 * (legdata->qd_wheel[0] + legdata->qd_wheel[1] - legdata->qd_wheel[2] - legdata->qd_wheel[3]) / 4.0;
    // wbc_data_->vBody_des[0] = vel * cos(yaw);
    // wbc_data_->vBody_des[1] = vel * sin(yaw);
    wbc_data_->aBody_des.setZero();
    if(1) {
        Vec3<float> orienvecdes(0, 0, attitude->rpy[2]);
        Eigen::Matrix<float, 4, 1> quatdes;
        Mat3<float> est_terrain_rot_matrix_ = posvelest->_stateEstimatorData.result->terrainRotationMatrix;
        quatdes = ori::rotationMatrixToQuaternion(ori::rpyToRotMat( orienvecdes ) * est_terrain_rot_matrix_.transpose());
        Eigen::Matrix<float, 4, 1> quatprev(attitude->quat[0], attitude->quat[1], attitude->quat[2], attitude->quat[3]);
        // The step below could avoid sign error
        if ((quatprev - quatdes).norm() > (quatprev + quatdes).norm())
        {
            quatdes = -quatdes;
        }
        Vec3<float> rpy_des_with_comp_ = ori::quatToRPY(quatdes);
        if (fabs(rpy_des_with_comp_[2] - attitude->rpy[2]) > 3.14)
        {
            rpy_des_with_comp_[2] = -rpy_des_with_comp_[2];
        }
        // _roll_des = rpy_des_with_comp_[0];
        pitch_compensate_ = rpy_des_with_comp_[1];
        // printf("p_c:%f\n", pitch_compensate_);
        if (pitch_compensate_ < -0.38)
            pitch_compensate_ = -0.38;
        else if (pitch_compensate_ > 0.38)
            pitch_compensate_ = 0.38;
      
  }


    wbc_data_->pBody_RPY_des[2] = attitude->rpy[2];
    wbc_data_->pBody_RPY_des[0] = 0.;
    wbc_data_->pBody_RPY_des[1] = pitch_compensate_;

    wbc_data_->vBody_Ori_des.setZero();

    for (size_t i(0); i < 4; ++i)
    {
        wbc_data_->pFoot_des[i] = posvelest->_stateEstimatorData.result->position + attitude->rot_body.transpose() * (legdata->GetHipLocation(i) + Vec3<float>(0, (i == 0 || i == 3) ? ABAD_LINK_LEN :-ABAD_LINK_LEN,0));
        wbc_data_->pFoot_des[i][2] = 0.0; 
        wbc_data_->vFoot_des[i].setZero();
        wbc_data_->aFoot_des[i].setZero();
        wbc_data_->Fr_des[i].setZero();
        wbc_data_->Fr_des[i][2] = body_weight_ / 4.;
        wbc_data_->contact_state[i] = 0.5;
    }
    wbc_ctrl_->run(wbc_data_, attitude, legdata, posvelest);

    Mat3<float> Kp_, Kd_;
    Kp_ << 3, 0, 0, 0, 3, 0, 0, 0, 3;
    Kd_ << 0.5, 0, 0, 0, 0.2, 0, 0, 0, 0.2;
    const float sign[4] = {1.0, 1.0, -1.0, -1.0};
    x_vel_des_ = ApplyVelocityMeetAccelationLimit<float>(x_vel_des_, x_vel_cmd_, -0.96, 0.96, -0.64, 0.64, 0.002);
    float v_wheel_cmd = x_vel_des_ / 0.0948;
    float des_leg_local_p[4] = {0.03, 0.03, -0.03, -0.03};
    for(int i = 0; i < 4; i++)
    {
        if(fabs(x_vel_des_) > 0.9)
          des_leg_local_p[i] = sign[i] * (0.03 + 0.15);
        else
          des_leg_local_p[i] = sign[i] * (0.03 + 0.15 * (fabs(x_vel_des_) / 0.9));
    }

    for (int i = 0; i < 4; i++)
    {
        Vec3<float> tau = Kp_ * (wbc_ctrl_->joint_pos[i] - Vec3<float>(legdata->q_abad[i], legdata->q_hip[i], legdata->q_knee[i])) + Kd_ * (wbc_ctrl_->joint_vel[i] - Vec3<float>(legdata->qd_abad[i], legdata->qd_hip[i], legdata->qd_knee[i])) + wbc_ctrl_->joint_tau[i];
        torque_set_[4 * i] = tau[0] + 25 * (-1.57 - legdata->q_abad[i]);
        torque_set_[4 * i + 1] = tau[1];
        torque_set_[4 * i + 2] = tau[2];
        float v_wheel_cmd_tmp = v_wheel_cmd + 8 * (des_leg_local_p[i] - legdata->foot_pos_local[i][0]);
        wheel_init_pos_abs_[i] += sign[i] * ( v_wheel_cmd_tmp ) * 0.002;
        torque_set_[4 * i + 3] = 6.5 * (wheel_init_pos_abs_[i] - legdata->q_wheel_abs[i]) + 0.5 * ((i < 2 ? v_wheel_cmd_tmp : -v_wheel_cmd_tmp) - legdata->qd_wheel[i]);
    }
}