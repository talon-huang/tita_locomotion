/*
 * @Author: mashicheng mashicheng@directdrivetec.com
 * @Date: 2024-01-05 13:26:44
 * @LastEditors: mashicheng mashicheng@directdrivetec.com
 * @LastEditTime: 2024-07-29 15:30:27
 * @FilePath: /tita_ros2/repos/apollo/src/libraries/locomotion/gait_control/src/task/Task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "task/Task.hpp"

void WheelQuadruped_Task::Run_Init()
{
    init_flag_ = true;
    down_flag_ = true;
    mpc_flag_ = true;
    wbc_flag_ = true;
    pos_est_->setData(&state_estimate_);
    pos_est_->set(legdata_, attidata_);
    pos_est_->setup();
    terrain_est_->setData(&state_estimate_);
    terrain_est_->set(legdata_, attidata_);
    terrain_est_->setup();
    enable_transition_ = true;
    current_mode_ = 0;

    ahrs = ahrs_init(500);
}

void WheelQuadruped_Task::Run_Update(float quat[4], float gyro[3], float accl[3], float joint_pos[16], float joint_vel[16])
{
    float quaternion[4] = {0.,0.,0.,0.};
    float quat_tmp[4] = {0.,0.,0.,0.};
    float accl_tmp[3] = {0.,0.,0.};
    for(int i = 0; i < 3; i++)
      accl_tmp[i] = accl[i] / 9.8;
    ahrs_update(ahrs, accl_tmp, gyro, 0, 0.002);
    ahrs_get_quaternion(ahrs, quaternion);
    // static int debug_count = 0;
    // if(debug_count++ % 200 == 0) {
    //   printf("arhs:%f,%f,%f,%f\n",quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    //   printf("org:%f,%f,%f,%f\n",quat[0], quat[1], quat[2], quat[3]);
    // }
    quat_tmp[0] = quaternion[3];
    for(int i = 0; i < 3; i++)
      quat_tmp[i + 1] = quaternion[i];
    attidata_->Update(quat_tmp, gyro, accl);
    legdata_->Update(joint_pos, joint_vel);
    pos_est_->run();
    terrain_est_->run();
}

void WheelQuadruped_Task::Run_Ctrl(int mode,VelGaitCmd vel_gait_cmd)
{ if(mode == 0)
    current_mode_ = 0;
  else if(mode == 3)
    current_mode_ = 3;
  else if(enable_transition_ && (mode != 2 || (mode == 2 && current_mode_ == 1) || (mode == 2 && current_mode_ == 4))
                             && (mode != 4 || (mode == 4 && current_mode_ == 1) || (mode == 4 && current_mode_ == 2)))
    current_mode_ = mode;
 
    if (current_mode_ != 1)
      init_flag_ = true;
    if (current_mode_ != 2)
      mpc_flag_ = true;
    if (current_mode_ != 3)
      down_flag_ = true;
    if (current_mode_ != 4)
      wbc_flag_ = true;

    if (current_mode_ == 0)
    {
      enable_transition_ = true;
      pos_est_->set_contact(Vec4<float>(0.5,0.5,0.5,0.5));
      for (int i = 0; i < 16; i++)
        tau_set_[i] = 0.0;
    }
    else if (current_mode_ == 1)
    {
      if (init_flag_)
      {
        init_flag_ = false;
        init_handle_->Package_Init();
        pos_est_->set_contact(Vec4<float>(0.5,0.5,0.5,0.5));
      }
      enable_transition_ = init_handle_->enable_transition_;
      init_handle_->Package_Run();
    }
    else if (current_mode_ == 2)
    {
      enable_transition_ = true;
      if (mpc_flag_)
      {
        mpc_flag_ = false;
        mpcloco_handle_->Package_Init(pos_est_, legdata_, attidata_);
      }
      mpcloco_handle_->Set_Cmd(vel_gait_cmd);
      mpcloco_handle_->Package_Run(attidata_, legdata_, pos_est_);
    } else if(current_mode_ == 3) {
      if (down_flag_)
      {
        down_flag_ = false;
        puredamper_handle_->Package_Init();
        pos_est_->set_contact(Vec4<float>(0.5,0.5,0.5,0.5));
      }
      enable_transition_ = puredamper_handle_->enable_transition_;
      puredamper_handle_->Package_Run();
    } else if(current_mode_ == 4) {
      enable_transition_ = (fabs(legdata_->foot_pos_local[0][0]) < 0.05 && fabs(legdata_->foot_pos_local[1][0]) < 0.05 && fabs(legdata_->foot_pos_local[2][0]) < 0.05 && fabs(legdata_->foot_pos_local[3][0]) < 0.05
                            && fabs(attidata_->rpy[1]) < 0.07);
      if (wbc_flag_)
      {
        wbc_flag_ = false;
        wbcstand_handle_->Package_Init(pos_est_, legdata_, attidata_);
      }
      static int debug_count = 0;
      // if(debug_count++ % 100 == 0)
      //   printf("vbody:%f,%f\n",pos_est_->_stateEstimatorData.result->vBody[0],pos_est_->_stateEstimatorData.result->vBody[1]);
      wbcstand_handle_->Set_VelPitch(2 * vel_gait_cmd.x_vel_cmd, 0.);
      wbcstand_handle_->Package_Run(attidata_, legdata_, pos_est_);
    }
    Safety_Check();
}

void WheelQuadruped_Task::Safety_Check()
{
  if(current_mode_ == 4)
  {
    if(fabs(attidata_->rpy[0]) > 0.15 || legdata_->foot_pos_local[0][0] < -0.12 || 
        legdata_->foot_pos_local[1][0] < -0.12 || legdata_->foot_pos_local[2][0] > 0.12 || legdata_->foot_pos_local[3][0] > 0.12)
      dangerous_flag_ = true;
    else
      dangerous_flag_ = false;
  } 
  else if(current_mode_ == 2)
  {
    if(fabs(attidata_->rpy[0]) > 0.15)
      dangerous_flag_ = true;
    else
      dangerous_flag_ = false;
  }
  else
  {
    dangerous_flag_ = false;
  }

}