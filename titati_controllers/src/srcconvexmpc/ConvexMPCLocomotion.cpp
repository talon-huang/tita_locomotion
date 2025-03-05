#include <iostream>

#include "incconvexmpc/ConvexMPCLocomotion.h"
#include "incconvexmpc/convexMPC_interface.h"
#include "common/Timer.h"
#include "incconvexmpc/Gait.h"
#include "common/Utilities.h"
//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH


////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  trotting(horizonLength+10, Vec4<int>(0,10,0,10), Vec4<int>(14,14,14,14),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(6,6,6,6),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength+6, Vec4<int>(0,8,0,8),Vec4<int>(11,11,11,11),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 400);
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++) {
    firstSwing[i] = true;
    firstStance[i] = true;
  }

   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();
}

void ConvexMPCLocomotion::initialize(AttitudeData* attitude_, LegData* legdata_, LinearKFPositionVelocityEstimator<float>* posvelest_){
  for(int i = 0; i < 4; i++) {
    firstSwing[i] = true;
    firstStance[i] = true;
  }
  firstRun = true;
  iterationCounter = 0;
  ini_count++;
  this->posvelest_ = posvelest_;
  this->attitude_ = attitude_;
  this->legdata_ = legdata_;
  for(int i = 0; i < 4; i++)
  {
    Fr_des[i].setZero();
    Fr_des_leg[i].setZero();
  }
  _yaw_turn_rate = 0;
  _x_vel_des = 0.;
  _y_vel_des = 0.;
  gaitNumber = 9;
}

void ConvexMPCLocomotion::set_vel_gait(float x_cmd, float y_cmd, float yaw_cmd, float gait_num) {
  _x_vel_cmd = 0*x_cmd;
  _y_vel_cmd = y_cmd;
  _yaw_turn_cmd = yaw_cmd;
  gaitNumber = gait_num;
}
void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(){

  _body_height = 0.28;

  float filter(0.1);
  if(1){
    // _yaw_turn_rate = 0;
    // _x_vel_cmd = 0.;
    // _y_vel_cmd = 0.;
    // static int cmd_cnt = 0;
    // cmd_cnt++;
    // if (cmd_cnt % 2000 < 500)
    //   x_vel_cmd = 1.1 * 0.0948;// * sin(3.1416*static_cast<float>(cmd_cnt % 2000)/500);
    // else if (cmd_cnt % 2000 < 1000)
    //   x_vel_cmd = 0;
    // else if (cmd_cnt % 2000 < 1500)
    //   x_vel_cmd = -1.1 * 0.0948;// * sin(3.1416*static_cast<float>(cmd_cnt % 2000 - 1000)/500);
    // else
    //   x_vel_cmd = 0.;
    // if (cmd_cnt < 500)
    //   x_vel_cmd = 5 * 0.0948 * sin(1.5708*static_cast<float>(cmd_cnt)/500);
    // else
    //   x_vel_cmd = 5 * 0.0948;
    // if (cmd_cnt % 1000 < 500)
    //   x_vel_cmd = 1.5*0.0948*sin(3.1416*static_cast<float>(cmd_cnt % 1000)/500);
    // else
    //   x_vel_cmd = -1.5*0.0948*sin(3.1416*static_cast<float>(cmd_cnt % 1000 - 500)/500);
    // y_vel_cmd = 0;
    _body_height += 0;
  }
  _x_vel_des = ApplyVelocityMeetAccelationLimit<float>(_x_vel_des, _x_vel_cmd, -0.48, 0.48, -0.12, 0.12, dt);
  _y_vel_des = ApplyVelocityMeetAccelationLimit<float>(_y_vel_des, _y_vel_cmd, -0.10, 0.10, -0.05, 0.05, dt);
  _yaw_turn_rate = ApplyVelocityMeetAccelationLimit<float>(_yaw_turn_rate, _yaw_turn_cmd, -1.2, 1.2, -0.5, 0.5, dt);

  // float k_x_cmd, k_yaw_cmd;
  // k_x_cmd = 1 - fabs(_x_vel_cmd) / 0.32;
  // k_x_cmd = (k_x_cmd < 0.001 ? 0 : sqrt(k_x_cmd));
  // k_yaw_cmd = 1 - fabs(_yaw_turn_cmd) / 1.2;
  // k_yaw_cmd = (k_yaw_cmd < 0.001 ? 0 : sqrt(k_yaw_cmd));
  // _yaw_turn_rate *= k_x_cmd;
  // _y_vel_des     *= (k_x_cmd*k_yaw_cmd);
  // _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _yaw_des = attitude_->rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = -0.;

}

// template<float>
void ConvexMPCLocomotion::run() {
  bool omniMode = false;

  // Command Setup
  _SetupCommand();
  // gaitNumber = 9;
  // if(gaitNumber >= 10) {
  //   gaitNumber -= 10;
  //   omniMode = true;
  // }

  // auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = posvelest_->_stateEstimatorData.result->position[0];//seResult.position[0];
    stand_traj[1] = posvelest_->_stateEstimatorData.result->position[1];//seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = attitude_->rpy[2];//seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 4)
    gait = &standing;
  // else if(gaitNumber == 2)
  //   gait = &pronking;
  // else if(gaitNumber == 3)
  //   gait = &random;
  // else if(gaitNumber == 4)
  //   gait = &standing;
  // else if(gaitNumber == 5)
  //   gait = &trotRunning;
  // else if(gaitNumber == 6)
  //   gait = &random2;
  // else if(gaitNumber == 7)
  //   gait = &random2;
  // else if(gaitNumber == 8)
  //   gait = &pacing;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);


  jumping.setIterations(27/2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  // jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
  //     data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  // if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
  //   gait = &jumping;
  //   recompute_timing(27/2);
  //   _body_height = _body_height_jumping;
  //   currently_jumping = true;

  // } else {
  recompute_timing(default_iterations_between_mpc);
  //   currently_jumping = false;
  // }

  if(_body_height < 0.02) {
    _body_height = 0.29;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : attitude_->rot_body.transpose() * v_des_robot;
  // Vec3<float> v_des_world = v_des_robot;
  Vec3<float> v_robot = posvelest_->_stateEstimatorData.result->vWorld;
  // Vec3<float> v_robot = Vec3<float>(0,0,0);

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - attitude_->rpy[1])/v_robot[0];
    // rpy_int[1] += dt*(_pitch_des - 0)/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - attitude_->rpy[0])/v_robot[1];
    // rpy_int[0] += dt*(_roll_des - 0)/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = 0;//v_robot[0] * rpy_int[1];
  rpy_comp[0] = 0;//v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking

  // float pFoot_tmp[4][3] = {0.23, -0.15, 0, 0.23, 0.15, 0, -0.23, -0.15, 0, -0.23, 0.15, 0};
  for(int i = 0; i < 4; i++) {
    pFoot[i] = posvelest_->_stateEstimatorData.result->position + 
      attitude_->rot_body.transpose() * (legdata_->GetHipLocation(i) + 
          legdata_->foot_pos_local[i]);
    // pFoot[i] = Vec3<float>(pFoot_tmp[i][0], pFoot_tmp[i][1], pFoot_tmp[i][2]);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = posvelest_->_stateEstimatorData.result->position[0];//seResult.position[0];
    world_position_desired[1] = posvelest_->_stateEstimatorData.result->position[1];//seResult.position[1];
    world_position_desired[2] = attitude_->rpy[2];//seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {
      if((gait->getContactState())[i] > 0.001)
        footSwingTrajectories[i].setHeight(0.0);
      else
        footSwingTrajectories[i].setHeight(0.05);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++) {
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
    stanceTimes[l] = gait->getCurrentStanceTime(dtMPC, l);
  }

  float side_sign[4] = {-1, 1, 0, 0};
  // float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  // float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  // float v_abs = std::fabs(v_des_robot[0]);

  static Vec3<float> relative_pos[4];
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    if(firstStance[i]) {
      stanceTimeRemaining[i] = stanceTimes[i];
    } else {
      stanceTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
    if((gait->getContactState())[i] > 0.001)
      footSwingTrajectories[i].setHeight(0.0);
    else
      footSwingTrajectories[i].setHeight(0.06);
    Vec3<float> offset(0, side_sign[i] * 0.04, 0);
    // Vec3<float> pRobotFrame_tmp[4];
    // pRobotFrame_tmp[0] << 0.23,0.15,0;
    // pRobotFrame_tmp[1] << 0.23,-0.15,0;
    // pRobotFrame_tmp[2] << -0.23,0.15,0;
    // pRobotFrame_tmp[3] << -0.23,-0.15,0;
#ifndef SIM_WEBOTS
    Vec3<float> pRobotFrame = (legdata_->GetHipLocation(i) + Vec3<float>(0, (i == 0 || i == 3) ? ABAD_LINK_LEN :-ABAD_LINK_LEN,0) + offset);
#else
    Vec3<float> pRobotFrame = (legdata_->GetHipLocation(i) + Vec3<float>(0, (i == 0 || i == 3) ? ABAD_LINK_LEN :-ABAD_LINK_LEN,0));
#endif
    // pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected = 
      ori::coordinateRotation(ori::CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = 0.0*_x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = posvelest_->_stateEstimatorData.result->position + attitude_->rot_body.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);// + Vec3<float>(0,i==0?-0.03:0.03,0));
    // Vec3<float> Pf = (pYawCorrected + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    // Vec3<float> des_vel_world = attitude_->rot_body.transpose() * des_vel;
    float pfx_rel = 1.0 * posvelest_->_stateEstimatorData.result->vWorld[0] * (.5 + 0.05) * stance_time +
      .03f*(posvelest_->_stateEstimatorData.result->vWorld[0] -v_des_world[0]) +
      (0.5f*posvelest_->_stateEstimatorData.result->position[2]/9.81f) * (posvelest_->_stateEstimatorData.result->vWorld[1]*_yaw_turn_rate);

    float pfy_rel = 1.0 * posvelest_->_stateEstimatorData.result->vWorld[1] * .5 * stance_time +
      .03f*(posvelest_->_stateEstimatorData.result->vWorld[1]-v_des_world[1]) +
      (0.5f*posvelest_->_stateEstimatorData.result->position[2]/9.81f) * (-posvelest_->_stateEstimatorData.result->vWorld[0]*_yaw_turn_rate);
    // float pfx_rel = 0 * (.5 + 0.05) * stance_time +
    //   .03f*(0-v_des_world[0]) +
    //   (0.25f*0/9.81f) * (0*_yaw_turn_rate);

    // float pfy_rel = 0 * .5 * stance_time * dtMPC +
    //   .03f*(0-v_des_world[1]) +
    //   (0.5f*0.25/9.81f) * (-0*_yaw_turn_rate);
    // if((gait->getContactState())[i] > 0.001) {
    //   if(firstStance[i] = true)
    //     relative_pos[i] = attitude_->rot_body * (pFoot[i] - posvelest_->_stateEstimatorData.result->position);
    //   Pf = posvelest_->_stateEstimatorData.result->position + attitude_->rot_body.transpose() * (relative_pos[i] + 0. * des_vel * stanceTimeRemaining[i]);
    //   pfx_rel = 0.0;
    //   pfy_rel = 0.0;
    // }
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    // Pf[2] = -0.003;
    Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);

  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  mpcTable = gait->getMpcTable();
  start_solve_mpc = false;
  if(iterationCounter % 10 == 1)
    start_solve_mpc = true;

  // updateMPCIfNeeded(mpcTable, omniMode, attitude_, posvelest_);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      firstStance[foot] = true;
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = attitude_->rot_body * (pDesFootWorld - posvelest_->_stateEstimatorData.result->position) 
        - legdata_->GetHipLocation(foot);
      Vec3<float> vDesLeg = attitude_->rot_body * (vDesFootWorld - posvelest_->_stateEstimatorData.result->vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(1){
        pFoot_des_leg[foot] = pDesLeg;
        vFoot_des_leg[foot] = vDesLeg;
        // Fr_des_leg[foot].setZero();
        // Update leg control command regardless of the usage of WBIC
        // data._legController->commands[foot].pDes = pDesLeg;
        // data._legController->commands[foot].vDes = vDesLeg;
        // data._legController->commands[foot].kpCartesian = Kp;
        // data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      if(firstStance[foot])
      {
        firstStance[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(contactState, stanceTimes[foot]);
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = attitude_->rot_body * (pDesFootWorld - posvelest_->_stateEstimatorData.result->position) 
        - legdata_->GetHipLocation(foot);
      Vec3<float> vDesLeg = attitude_->rot_body * (vDesFootWorld - posvelest_->_stateEstimatorData.result->vWorld);
      // Vec3<float> getHipLocation[4];
      // getHipLocation[0] << 0.23,0.15,0;
      // getHipLocation[1] << 0.23,-0.15,0;
      // getHipLocation[2] << -0.23,0.15,0;
      // getHipLocation[3] << -0.23,-0.15,0;
      // Vec3<float> pDesLeg = (pDesFootWorld - Vec3<float>(0,0,0.25)) - getHipLocation[foot];//data._quadruped->getHipLocation(foot);
      // Vec3<float> vDesLeg = (vDesFootWorld - Vec3<float>(0,0,0));
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(1){
        pFoot_des_leg[foot] = pDesLeg;
        vFoot_des_leg[foot] = vDesLeg;
        // Fr_des_leg[foot] = f_ff[foot];
        // data._legController->commands[foot].pDes = pDesLeg;
        // data._legController->commands[foot].vDes = vDesLeg;
        // data._legController->commands[foot].kpCartesian = Kp_stance;
        // data._legController->commands[foot].kdCartesian = Kd_stance;

        // data._legController->commands[foot].forceFeedForward = f_ff[foot];
        // data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        // data._legController->commands[foot].pDes = pDesLeg;
        // data._legController->commands[foot].vDes = vDesLeg;
        // data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        // data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  // data._stateEstimator->setContactPhase(se_contactState);
  posvelest_->set_contact(se_contactState);
  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = _pitch_des; 
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update


}

// template<>
// void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
//   (void)data;
//   printf("call to old CMPC with double!\n");

// }
void ConvexMPCLocomotion::solve_another_thread() {
    updateMPCIfNeeded(false);
}


void ConvexMPCLocomotion::updateMPCIfNeeded(bool omniMode) {
  //iterationsBetweenMPC = 30;
  if(1)
  {
    // auto seResult = data._stateEstimator->getResult();
    float* p = posvelest_->_stateEstimatorData.result->position.data();
    // float p[3] = {0,0,0.25};

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : attitude_->rot_body.transpose() * v_des_robot;
    // Vec3<float> v_des_world = v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
        _pitch_des,    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = attitude_->rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    // if(_parameters->cmpc_use_sparse > 0.5) {
    //   solveSparseMPC(mpcTable, data);
    // } else {
    //   solveDenseMPC(mpcTable, data);
    // }
    solveDenseMPC();
    printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC() {
  // auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {1.25, 1.25, 2, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = attitude_->rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = posvelest_->_stateEstimatorData.result->position.data();
  float* v = posvelest_->_stateEstimatorData.result->vWorld.data();
  float* w = attitude_->omega_world.data();
  float* q = attitude_->quat.data();
  // float p[3] = {0,0,0.25};
  // float v[3] = {0,0,0};
  // float w[3] = {0,0,0};
  // float q[4] = {1,0,0,0};

  float r[12];
  Vec3<float> com_offset(0.0*cos(yaw),0.0*sin(yaw),0);
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - (posvelest_->_stateEstimatorData.result->position[i/4] + com_offset[i/4]);
  // float r[12];
  // float pos_tmp[3] = {0,0,0.25};
  // for(int i = 0; i < 12; i++)
  //   r[i] = pFoot[i%4][i/4]  - pos_tmp[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(posvelest_->_stateEstimatorData.result->vWorld[0], posvelest_->_stateEstimatorData.result->vWorld[1], 0);
  // Vec3<float> vxy(0, 0, 0);

  // Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC,horizonLength,0.4,400);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    // x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
    x_comp_integral += 0.3 * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  // update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
  //     _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  // Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -attitude_->rot_body * f;
    Fr_des_leg[leg] = f_ff[leg];
    // f_ff[leg] = -f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}



