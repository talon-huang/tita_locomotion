#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

// #include <FSM_States/ControlFSMData.h>
#include "incdynamics/FloatingBaseModel.h"
#include "incdynamics/Quadruped.h"
#include "common/cppTypes.h"
#include "incwbc/WBIC/WBIC.hpp"
#include "incwbc/WBIC/KinWBC.hpp"

// #include <lcm/lcm-cpp.hpp>
// #include "wbc_test_data_t.hpp"
#include "common/AttitudeData.h"
#include "common/LegData.h"
#include "incestimator/PositionVelocityEstimator.h"

#define WBCtrl WBC_Ctrl<T>

class MIT_UserParameters;

template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(FloatingBaseModel<T> model);
    virtual ~WBC_Ctrl();

    void run(void * input, AttitudeData* attidata, LegData* legdata, LinearKFPositionVelocityEstimator<float> *posvelest);
    void setFloatingBaseWeight(const T & weight){
      _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
    }
    Vec3<T> joint_pos[4];
    Vec3<T> joint_vel[4];
    Vec3<T> joint_tau[4];
  protected:
    virtual void _ContactTaskUpdate(void * input) = 0;
    // virtual void _ContactTaskUpdateTEST(void * input, ControlFSMData<T> & data){
    //   (void)input;
    //   (void)data;
    // }
    virtual void _LCM_PublishData(){}
    void _UpdateModel(AttitudeData* attidata, LegData* legdata, LinearKFPositionVelocityEstimator<float> *posvelest);
    void _UpdateLegCMD();
    void _ComputeWBC();

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;

    FloatingBaseModel<T> _model;
    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    FBModelState<T> _state;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<T> _Kp_joint, _Kd_joint;
    //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

    unsigned long long _iter;

    // lcm::LCM _wbcLCM;
    // wbc_test_data_t _wbc_data_lcm;
};
#endif
