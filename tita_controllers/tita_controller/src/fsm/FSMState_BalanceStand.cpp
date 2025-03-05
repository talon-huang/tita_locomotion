#include "fsm/FSMState_BalanceStand.h"
FSMState_BalanceStand::FSMState_BalanceStand(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::BALANCE_STAND, "balance_stand")
{
}

void FSMState_BalanceStand::enter() { _data->state_command->firstRun = true; }

void FSMState_BalanceStand::run()
{
  _data->state_command->convertToStateCommands();
  _data->pinocchio_model->updateWBC();

  _data->low_cmd->kp = Eigen::Map<DVec<scalar_t>>(
    _data->params->wbc_joint_kp.data(), _data->params->wbc_joint_kp.size());
  _data->low_cmd->kd = Eigen::Map<DVec<scalar_t>>(
    _data->params->wbc_joint_kd.data(), _data->params->wbc_joint_kd.size());
}

void FSMState_BalanceStand::exit()
{
  // _data->desired_state_command_->exit();
  // counter = 0;
  // _data->_interface->zeroCmdPanel();
}

FSMStateName FSMState_BalanceStand::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name) {
    case FSMStateName::BALANCE_STAND:
      break;
    case FSMStateName::TRANSFORM_DOWN:
      this->_nextStateName = FSMStateName::TRANSFORM_DOWN;
      break;
    case FSMStateName::PASSIVE:  // normal c
      this->_nextStateName = FSMStateName::PASSIVE;
      break;
    default:
      break;
  }
  return this->_nextStateName;
}
