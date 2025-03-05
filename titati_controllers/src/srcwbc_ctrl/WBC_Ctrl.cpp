#include "incwbc_ctrl/WBC_Ctrl.hpp"
// #include <Utilities_print.h>
// #include <Utilities/Timer.h>

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
  _full_config(cheetah::num_act_joint + 7),
  _tau_ff(cheetah::num_act_joint),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint)
  // _wbcLCM("udpm://239.255.76.67:7667?ttl=255")
{
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 10.);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  // _wbic_data->_W_floating[1] = 0.2;
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);
  _state.q.resize(cheetah::num_act_joint, 1);
  _state.qd.resize(cheetah::num_act_joint, 1);
  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  // TEST
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, AttitudeData* attidata, LegData* legdata, LinearKFPositionVelocityEstimator<float> *posvelest){
  ++_iter;

  // Update Model
  _UpdateModel(attidata, legdata, posvelest);

  // Task & Contact Update
  _ContactTaskUpdate(input);

  // WBC Computation
  _ComputeWBC();
  
  // TEST
  //T dt(0.002);
  //for(size_t i(0); i<12; ++i){
    //_des_jpos[i] = _state.q[i] + _state.qd[i] * dt + 0.5 * _wbic_data->_qddot[i+6] * dt * dt;
    //_des_jvel[i] = _state.qd[i] + _wbic_data->_qddot[i+6]*dt;
  //}

  //_ContactTaskUpdateTEST(input, data);
  //_ComputeWBC();
  // END of TEST

  // Update Leg Command
  _UpdateLegCMD();

  // LCM publish
  // _LCM_PublishData();
}



template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD() {
  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    if(leg > 1)
    {
      joint_pos[leg] << -_des_jpos[cheetah::num_leg_joint * leg] - 1.5708, -_des_jpos[cheetah::num_leg_joint * leg +1], -_des_jpos[cheetah::num_leg_joint * leg + 2];
      joint_vel[leg] << -_des_jvel[cheetah::num_leg_joint * leg], -_des_jvel[cheetah::num_leg_joint * leg  + 1], -_des_jvel[cheetah::num_leg_joint * leg + 2];
      joint_tau[leg] << -_tau_ff[cheetah::num_leg_joint * leg], -_tau_ff[cheetah::num_leg_joint * leg + 1], -_tau_ff[cheetah::num_leg_joint * leg + 2];
    }
    else
    {
      joint_pos[leg] << _des_jpos[cheetah::num_leg_joint * leg] - 1.5708, _des_jpos[cheetah::num_leg_joint * leg +1], _des_jpos[cheetah::num_leg_joint * leg + 2];
      joint_vel[leg] << _des_jvel[cheetah::num_leg_joint * leg], _des_jvel[cheetah::num_leg_joint * leg  + 1], _des_jvel[cheetah::num_leg_joint * leg + 2];
      joint_tau[leg] << _tau_ff[cheetah::num_leg_joint * leg], _tau_ff[cheetah::num_leg_joint * leg + 1], _tau_ff[cheetah::num_leg_joint * leg + 2];
    }
  }

  // LegControllerCommand<T> * cmd = data._legController->commands;
  // //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;

  // for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
  //   cmd[leg].zero();
  //   for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
  //     cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
  //     cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
  //     cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];

  //       cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
  //       cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
       
  //      //if(contact[leg] > 0.){ // Contact
  //       //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
  //       //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
  //     //}else{
  //       //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint_swing[jidx];
  //       //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint_swing[jidx];
  //     //}

  //   }
  // }


  // // Knee joint non flip barrier
  // for(size_t leg(0); leg<4; ++leg){
  //   if(cmd[leg].qDes[2] < 0.3){
  //     cmd[leg].qDes[2] = 0.3;
  //   }
  //   if(data._legController->datas[leg].q[2] < 0.3){
  //     T knee_pos = data._legController->datas[leg].q[2]; 
  //     cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
  //   }
  // }
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(AttitudeData* attidata, LegData* legdata, LinearKFPositionVelocityEstimator<float> *posvelest){
  (void)legdata;
  _state.bodyOrientation << attidata->quat[0], attidata->quat[1], attidata->quat[2], attidata->quat[3];
  _state.bodyPosition = posvelest->_stateEstimatorData.result->position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = attidata->omega_body[i];
    _state.bodyVelocity[i+3] = posvelest->_stateEstimatorData.result->vBody[i];

    for(size_t leg(0); leg<4; ++leg){
      if(leg > 1)
      {
        _state.q[3*leg + i] = (i == 0 ? -(legdata->q_abad[leg] + 1.5708): (i == 1 ? -legdata->q_hip[leg] : -legdata->q_knee[leg]));
        _state.qd[3*leg + i] = (i == 0 ? -legdata->qd_abad[leg] : (i == 1 ? -legdata->qd_hip[leg] : -legdata->qd_knee[leg]));
      }
      else
      {
        _state.q[3*leg + i] = (i == 0 ? legdata->q_abad[leg] + 1.5708: (i == 1 ? legdata->q_hip[leg] : legdata->q_knee[leg]));
        _state.qd[3*leg + i] = (i == 0 ? legdata->qd_abad[leg] : (i == 1 ? legdata->qd_hip[leg] : legdata->qd_knee[leg]));
      }
      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  // pretty_print(_state.bodyOrientation, std::cout, "ori");
  // pretty_print(_state.bodyPosition, std::cout, "pos");
  // pretty_print(_state.bodyVelocity, std::cout, "vel");
  // pretty_print(_state.q, std::cout, "sq");
  // pretty_print(_state.qd, std::cout, "sqd");
  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();
}


template class WBC_Ctrl<float>;
// template class WBC_Ctrl<double>;
