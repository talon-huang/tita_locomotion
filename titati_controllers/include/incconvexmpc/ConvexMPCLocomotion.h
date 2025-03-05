#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include "incconvexmpc/FootSwingTrajectory.h"
#include "common/cppTypes.h"
#include "incconvexmpc/Gait.h"
#include "common/orientation_tools.h"
#include <cstdio>
#include "incestimator/PositionVelocityEstimator.h"
#include "common/AttitudeData.h"
#include "common/LegData.h"

using Eigen::Array4f;
using Eigen::Array4i;

// template <typename T>
// struct CMPC_Result
// {
//   Vec4<T> contactPhase;
// };

// struct CMPC_Jump
// {
//   static constexpr int START_SEG = 6;
//   static constexpr int END_SEG = 0;
//   static constexpr int END_COUNT = 2;
//   bool jump_pending = false;
//   bool jump_in_progress = false;
//   bool pressed = false;
//   int seen_end_count = 0;
//   int last_seg_seen = 0;
//   int jump_wait_counter = 0;

//   void debug(int seg)
//   {
//     (void)seg;
//     // printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
//   }

//   void trigger_pressed(int seg, bool trigger)
//   {
//     (void)seg;
//     if (!pressed && trigger)
//     {
//       if (!jump_pending && !jump_in_progress)
//       {
//         jump_pending = true;
//         // printf("jump pending @ %d\n", seg);
//       }
//     }
//     pressed = trigger;
//   }

//   bool should_jump(int seg)
//   {
//     debug(seg);

//     if (jump_pending && seg == START_SEG)
//     {
//       jump_pending = false;
//       jump_in_progress = true;
//       // printf("jump begin @ %d\n", seg);
//       seen_end_count = 0;
//       last_seg_seen = seg;
//       return true;
//     }

//     if (jump_in_progress)
//     {
//       if (seg == END_SEG && seg != last_seg_seen)
//       {
//         seen_end_count++;
//         if (seen_end_count == END_COUNT)
//         {
//           seen_end_count = 0;
//           jump_in_progress = false;
//           // printf("jump end @ %d\n", seg);
//           last_seg_seen = seg;
//           return false;
//         }
//       }
//       last_seg_seen = seg;
//       return true;
//     }

//     last_seg_seen = seg;
//     return false;
//   }
// };

class ConvexMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc);
  void initialize(AttitudeData* attitude_, LegData* legdata_, LinearKFPositionVelocityEstimator<float>* posvelest_);

  void run();
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;
  
  Vec3<float> pFoot_des_leg[4];
  Vec3<float> vFoot_des_leg[4];
  Vec3<float> Fr_des_leg[4];
  
  bool start_solve_mpc = false;
  void solve_another_thread();
  void set_vel_gait(float x_cmd, float y_cmd, float yaw_cmd, float gait_num);
private:
  void _SetupCommand();

  float _yaw_turn_cmd;
  float _yaw_turn_rate;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;
  float _x_vel_cmd = 0.;
  float _y_vel_cmd = 0.;

  // High speed running
  // float _body_height = 0.34;
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(bool omniMode);
  void solveDenseMPC();
  void solveSparseMPC(int *mpcTable);
  void initSparseMPC();
  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  Vec4<float> stanceTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  bool firstStance[4];
  float swingTimeRemaining[4];
  float stanceTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  // CMPC_Result<float> result;
  float trajAll[12 * 36];

  // CMPC_Jump jump_state;
  int       ini_count = 0;
  int* mpcTable;
  LinearKFPositionVelocityEstimator<float>* posvelest_;
  AttitudeData* attitude_;
  LegData* legdata_;
};

#endif // CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
