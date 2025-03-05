#include <chrono>
#include <iostream>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

int main(int /* argc */, char ** /* argv */)
{
  std::string urdf_path = "/usr/share/robot_description/tita/urdf/robot.urdf";
  std::vector<std::string> wheel_joint_name = {"joint_left_leg_4", "joint_right_leg_4"};
  std::vector<std::string> contact_frame_names = {"left_contact", "right_contact"};

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);

  // Add two contact frame
  pinocchio::Frame frame;
  frame.placement = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.095, 0.0, 0.0));
  frame.type = pinocchio::FIXED_JOINT;  // 帧的类型（固定）
  for (auto i = 0; i < contact_frame_names.size(); i++) {
    frame.parent = model.getJointId(wheel_joint_name[i]);  // 关节 ID
    frame.name = contact_frame_names[i];                   // 帧的名称
    model.addFrame(frame);
  }
  pinocchio::Data data(model);

  auto FRAME_ID = model.getFrameId(contact_frame_names[0]);

  Eigen::VectorXd q = pinocchio::neutral(model);
  q << 0, 1.2, -2.4, 0, 0, 0, 0, 0;
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);

  pinocchio::forwardKinematics(model, data, q, qdot);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::SE3 oMf = data.oMf[FRAME_ID];

  std::cout << "oMf: " << oMf.translation().transpose() << std::endl;

  Eigen::Vector3d pos_des = oMf.translation() + Eigen::Vector3d(0.1, 0.1, 0.0);
  
  const double eps = 1e-3;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-0;

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  bool success = false;
  Eigen::Vector3d err;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::computeFrameJacobian(model, data, q, FRAME_ID, pinocchio::LOCAL_WORLD_ALIGNED, J);  // J in joint frame
    // std::cout << "J:\n" << J << std::endl;
    Eigen::MatrixXd Jtop = J.block(0, 0, 3, 4); 
    // std::cout << "Jtop:\n" << Jtop << std::endl;
    
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(Jtop.rows(), Jtop.rows());
    auto Jtop_pi = Jtop.transpose() * (Jtop * Jtop.transpose() /*+ damp * damp * identity*/).inverse();

    pinocchio::updateFramePlacements(model, data);
    err = pos_des - data.oMf[FRAME_ID].translation();
    q.segment(0, 4) += Jtop_pi * err;

    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= IT_MAX) {
      success = false;
      break;
    }

    if (!(i % 10)) std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;
  std::cout << "程序运行时间: " << duration.count() << " 秒" << std::endl;


  if (success) {
    std::cout << "Convergence achieved!" << std::endl;
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::computeFrameJacobian(model, data, q, FRAME_ID, J);  // J in joint frame
    std::cout << "final position:\n " << data.oMf[FRAME_ID].translation() << std::endl;
  } else {
    std::cout
      << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
      << std::endl;
  }

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;
}