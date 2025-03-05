/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-01-05 13:26:44
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-04-10 13:34:20
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/include/task/WbcStand.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef WBCSTAND_HPP
#define WBCSTAND_HPP

#include "common/LegData.h"
#include "common/AttitudeData.h"
#include "incestimator/PositionVelocityEstimator.h"
#include "incwbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp"
#include "incdynamics/MiniCheetah.h"

class WbcStand_Handle
{
public:
    WbcStand_Handle(float* torque_set, Quadruped<float>* quadruped):torque_set_(torque_set)
    {
        wbc_ctrl_ = new LocomotionCtrl<float>(quadruped->buildModel());
        wbc_data_ = new LocomotionCtrlData<float>(); 
    }
    ~WbcStand_Handle()
    {
        delete wbc_ctrl_;
        delete wbc_data_;
    }

    void Package_Init(LinearKFPositionVelocityEstimator<float>* pos_est, LegData* legdata, AttitudeData* attidata);
    void Package_Run(AttitudeData* attitude, LegData* legdata, LinearKFPositionVelocityEstimator<float>* posvelest);
    void Set_VelPitch(float x_vel_cmd, float pitch_cmd)
    {
        if(x_vel_cmd > 0.96)
          x_vel_cmd_ = 0.96;
        else if(x_vel_cmd < -0.96)
          x_vel_cmd_ = -0.96;
        else
          x_vel_cmd_ = x_vel_cmd;
        
        if(pitch_cmd > 0.3)
          pitch_cmd_ = 0.3;
        else if(pitch_cmd < -0.3)
          pitch_cmd_ = -0.3;
        else
          pitch_cmd_ = pitch_cmd;
    } 
private:
    float* torque_set_;
    float wheel_init_pos_abs_[4];

    // Quadruped<float> quadruped_;
    // FloatingBaseModel<float> model_;
    WBC_Ctrl<float> * wbc_ctrl_;
    LocomotionCtrlData<float> * wbc_data_;

    Vec3<float> ini_body_pos_;
    Vec3<float> ini_body_ori_rpy_;
    float body_weight_;

    float x_vel_cmd_;
    float x_vel_des_;
    float pitch_cmd_;
    float pitch_compensate_;

};


#endif