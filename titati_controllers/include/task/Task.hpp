/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-03-26 19:05:34
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-07-29 14:28:32
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/include/task/Task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef WQ_TASK_HPP
#define WQ_TASK_HPP

#include "task/Init.hpp"
#include "common/LegData.h"
#include "common/AttitudeData.h"
#include "incestimator/PositionVelocityEstimator.h"
#include "incestimator/TerrainEstimator.h"
#include "incconvexmpc/ConvexMPCLocomotion.h"
#include "task/MpcLoco.hpp"
#include "task/PureDamper.hpp"
#include "task/WbcStand.hpp"
#include "ahrs.h"

class WheelQuadruped_Task {
public:
  WheelQuadruped_Task(float* tau_set):tau_set_(tau_set)
  {
    quadruped_ = buildMiniCheetah<float>();
    legdata_ = new LegData();
    init_handle_ = new Init_Handle(tau_set_, legdata_);
    puredamper_handle_ = new PureDamper_Handle(tau_set_, legdata_);
    mpcloco_handle_ = new  MpcLoco_Handle(tau_set_, &quadruped_);
    attidata_ = new AttitudeData();
    pos_est_ = new LinearKFPositionVelocityEstimator<float>();
    terrain_est_ = new TerrainEstimator<float>();
    wbcstand_handle_ = new WbcStand_Handle(tau_set_, &quadruped_);
  }
  ~WheelQuadruped_Task()
  {
    delete legdata_;
    delete init_handle_;
    delete puredamper_handle_;
    delete mpcloco_handle_;
    delete attidata_ ;
    delete pos_est_;
    delete terrain_est_;
    delete wbcstand_handle_;
  }

  void Run_Init();
  void Run_Update(float quat[4], float gyro[3], float accl[3], float joint_pos[16], float joint_vel[16]);
  void Run_Ctrl(int mode, VelGaitCmd vel_gait_cmd);

  int Get_Current_Mode() {return current_mode_;}
  bool Get_DangerousPos_Flag() {return dangerous_flag_;};
private:
  LegData* legdata_;
  float* tau_set_;
  Init_Handle* init_handle_;
  bool init_flag_ = true;
  PureDamper_Handle* puredamper_handle_;
  bool down_flag_ = true;
  MpcLoco_Handle* mpcloco_handle_;
  bool mpc_flag_ = true;
  WbcStand_Handle* wbcstand_handle_;
  bool wbc_flag_ = true;
  AttitudeData* attidata_;
  LinearKFPositionVelocityEstimator<float>* pos_est_;
  TerrainEstimator<float>* terrain_est_;

  int current_mode_ = 0;
  bool enable_transition_ = true;

  Quadruped<float> quadruped_;
  StateEstimate<float> state_estimate_;

  bool dangerous_flag_ = false;
  void Safety_Check();

  void* ahrs;
};

#endif