#ifndef MPCLOCO_HPP
#define MPCLOCO_HPP

#include "common/LegData.h"
#include "common/AttitudeData.h"
#include "common/Utilities.h"
#include "incestimator/PositionVelocityEstimator.h"
#include "incconvexmpc/ConvexMPCLocomotion.h"
#include "incwbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp"
#include "incdynamics/MiniCheetah.h"
#include <semaphore.h>
#include <thread>
#include <unistd.h>

struct VelGaitCmd
{
  float yaw_turn_rate;
  float x_vel_cmd;
  float y_vel_cmd;
  int gait_number;
  bool use_wbc;
  VelGaitCmd():
  yaw_turn_rate(0.),
  x_vel_cmd(0.),
  y_vel_cmd(0.),
  gait_number(9),
  use_wbc(false){}
};

class MpcLoco_Handle 
{

public:
    MpcLoco_Handle(float* torque_set, Quadruped<float>* quadruped):torque_set_(torque_set)
    {
        mpc_test_ = new ConvexMPCLocomotion(0.002, 15);
        // quadruped_ = buildMiniCheetah<float>();
        // model_ = quadruped_.buildModel();
        wbc_ctrl_ = new LocomotionCtrl<float>(quadruped->buildModel());
        wbc_data_ = new LocomotionCtrlData<float>();
        sem_init(&mpc_start_, 0, 0);
        auto mpc_task = [&]() -> void
        {
            while (1)
            {
                sem_wait(&mpc_start_);
                mpc_test_->solve_another_thread();
                usleep(20);
                // leg_locomotion_drive_handle.Package_Run(pos_est, legdata, attidata);
            }
        };
        mpc_thread_ = std::thread(mpc_task);
     
    }
    ~MpcLoco_Handle()
    {
        delete mpc_test_;
        delete wbc_ctrl_;
        delete wbc_data_;
    }
    void Package_Init(LinearKFPositionVelocityEstimator<float>* pos_est, LegData* legdata, AttitudeData* attidata);
    void Package_Run(AttitudeData* attitude, LegData* legdata, LinearKFPositionVelocityEstimator<float>* posvelest);
    void Set_Cmd(VelGaitCmd vel_gait_cmd) {
        vel_gait_cmd_ = vel_gait_cmd;

        if(vel_gait_cmd_.x_vel_cmd > 0.56)
          vel_gait_cmd_.x_vel_cmd = 0.56;
        else if(vel_gait_cmd_.x_vel_cmd < -0.56)
          vel_gait_cmd_.x_vel_cmd = -0.56;
        
        if(vel_gait_cmd_.y_vel_cmd > 0.1)
          vel_gait_cmd_.y_vel_cmd = 0.1;
        else if(vel_gait_cmd_.y_vel_cmd < -0.1)
          vel_gait_cmd_.y_vel_cmd = -0.1;

        if(vel_gait_cmd_.yaw_turn_rate > 1.2)
          vel_gait_cmd_.yaw_turn_rate = 1.2;
        else if(vel_gait_cmd_.yaw_turn_rate < -1.2)
          vel_gait_cmd_.yaw_turn_rate = -1.2;
        
        if(vel_gait_cmd_.gait_number != 5 && vel_gait_cmd_.gait_number != 9)
          vel_gait_cmd_.gait_number = 9;
    }
private:
    float* torque_set_;
    ConvexMPCLocomotion* mpc_test_;
    float wheel_init_pos_[4];

    // Quadruped<float> quadruped_;
    // FloatingBaseModel<float> model_;
    WBC_Ctrl<float> * wbc_ctrl_;
    LocomotionCtrlData<float> * wbc_data_;

    std::thread mpc_thread_;
    sem_t mpc_start_;

    bool first_stance_[4] = {true,true,true,true};
    float wheel_init_abs_pos_[4] = {0.,0.,0.,0.};
    int wheel_stance_count_[4] = {0,0,0,0};

    VelGaitCmd vel_gait_cmd_;
    float x_vel_cmd_;
};

#endif