// #ifndef LQR_BASE_HPP
// #define LQR_BASE_HPP
// #include "common/RobotParameters.h"
// #include "common/enumClass.h"
// #include "lqr/riccati_solver.h"

// class LqrBase
// {
// public:
//   LqrBase(
//     std::shared_ptr<RobotControlParameters> params,
//     std::shared_ptr<WheelLeggedData> wheel_legged_data)
//   {
//     params_ = params;
//     wheel_legged_data_ = wheel_legged_data;
//   }

//   void update()
//   {
//     // check
//     auto r_comz = wheel_legged_data_->com_position_relative(point::Z);
//     Eigen::MatrixXd A_qr, B_qr;
//     Eigen::MatrixXd Q_qr, R_qr;

//     int lqr_states_size, lqr_inputs_size;
//     lqr_states_size = 4, lqr_inputs_size = 1;

//     A_qr.setZero(lqr_states_size, lqr_states_size);
//     B_qr.setZero(lqr_states_size, lqr_inputs_size);
//     Q_qr.setZero(lqr_states_size, lqr_states_size);
//     R_qr.setZero(lqr_inputs_size, lqr_inputs_size);

//     // some varible
//     const scalar_t g = 9.81f;
//     Q_qr(0, 0) = params_->lqr_q0, Q_qr(1, 1) = params_->lqr_q1, Q_qr(2, 2) = params_->lqr_q2,
//             Q_qr(3, 3) = params_->lqr_q3;
//     R_qr(0, 0) = params_->lqr_r0;

//     A_qr(0, 1) = A_qr(2, 3) = 1;
//     r_comz = r_comz > 0.01 ? r_comz : 0.01;
//     A_qr(1, 2) = g / r_comz;  // TODO:
//     B_qr(3, 0) = 1;



//     // solve riccati equation
//     Eigen::MatrixXd P_qr;
//     solveRiccatiArimotoPotter(A_qr, B_qr, Q_qr, R_qr, P_qr);
//     wheel_legged_data_->K_ = R_qr.inverse() * B_qr.transpose() * P_qr;
//   }

// private:
//   std::shared_ptr<RobotControlParameters> params_;
//   std::shared_ptr<WheelLeggedData> wheel_legged_data_;
// };

// #endif
