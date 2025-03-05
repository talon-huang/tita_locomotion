#include <chrono>
#include <iostream>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

#define PRINT_MAT(X) std::cout << #X << ":\n" << std::fixed << std::setprecision(6) << X << "\n\n"

Eigen::Quaterniond eularToQuat(Eigen::Vector3d eular)
{
  Eigen::Quaterniond quat = Eigen::AngleAxisd(eular(2), Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(eular(1), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(eular(0), Eigen::Vector3d::UnitX());
  return quat;
}

Eigen::MatrixXd getSMatrix(Eigen::Matrix3d Rbc)
{
  Eigen::MatrixXd S(2 * Rbc.rows(), 2 * Rbc.cols());
  S << -0.5 * Rbc.transpose(), -0.5 * Rbc.transpose(), 0.5 * Rbc.transpose(), -0.5 * Rbc.transpose();
  return S;
}

int main(int /* argc */, char ** /* argv */)
{
  std::cout << std::fixed << std::setprecision(4);

  std::string urdf_path = "/usr/share/robot_description/tita/urdf/robot.urdf";
  std::vector<std::string> wheel_joint_name = {"joint_left_leg_4", "joint_right_leg_4"};
  std::vector<std::string> contact_frame_names = {"left_contact", "right_contact"};

  pinocchio::Model model, model_fix;
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model);
  pinocchio::urdf::buildModel(urdf_path, model_fix);

  // Add two contact frame
  pinocchio::Frame frame;
  Eigen::Matrix3d rot;
  rot << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  frame.placement = pinocchio::SE3(rot, Eigen::Vector3d(0.095, 0.0, 0.0));
  frame.type = pinocchio::FIXED_JOINT;  // 帧的类型（固定）
  for (size_t i = 0; i < contact_frame_names.size(); i++) {
    frame.parent = model.getJointId(wheel_joint_name[i]);  // 关节 ID
    frame.name = contact_frame_names[i];                   // 帧的名称
    model.addFrame(frame);
    frame.parent = model_fix.getJointId(wheel_joint_name[i]);  // 关节 ID
    model_fix.addFrame(frame);
  }
  pinocchio::Data data(model);
  pinocchio::Data data_fix(model_fix);

  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::Vector3d pwb, eular;
  pwb << 0.0, 0.0, 0.0;
  eular << 0.055, 0.0 / 4, 0 / 2;

  Eigen::Quaterniond quat = eularToQuat(eular);
  Eigen::Matrix3d Rbw = quat.toRotationMatrix().transpose();
  Eigen::Matrix3d Rcw =
    Eigen::AngleAxisd(eular(2), Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose();
  Eigen::Matrix3d Rbc = Rbw * Rcw.transpose();

  std::cout << " Rbc: \n" << Rbc << "\n " << std::endl;

  // clang-format off
  q << pwb, 
       quat.x(), quat.y(), quat.z(), quat.w(),
       -0.055, 0.916, -1.836, 0.916, -0.055, 0.949, -1.90, 0.949;
  // clang-format on
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.nv);

  std::vector<Eigen::Vector3d> contact_pos_world, contact_pos_body;
  contact_pos_world.resize(2);
  contact_pos_body.resize(2);

  std::vector<Eigen::Matrix3d> contact_rotation_world, contact_rotation_body;
  contact_rotation_world.resize(2);
  contact_rotation_body.resize(2);

  pinocchio::forwardKinematics(model, data, q, qdot);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::forwardKinematics(model_fix, data_fix, q.tail(model_fix.nq), qdot.tail(model_fix.nv));
  pinocchio::updateFramePlacements(model_fix, data_fix);

  std::cout << "------ contact position and rotation ------ " << std::endl;
  for (size_t i = 0; i < contact_frame_names.size(); i++) {
    auto FRAME_ID = model.getFrameId(contact_frame_names[i]);
    contact_pos_world[i] = data.oMf[FRAME_ID].translation();
    contact_rotation_world[i] = data.oMf[FRAME_ID].rotation();
    auto FRAME_ID_fix = model_fix.getFrameId(contact_frame_names[i]);
    contact_pos_body[i] =
      data_fix.oMf[FRAME_ID_fix].translation();  // Rbw * (contact_pos_world[i] - pwb);
    contact_rotation_body[i] =
      data_fix.oMf[FRAME_ID_fix].rotation();  // Rbw * contact_rotation_world[i];

    std::cout << " " << contact_frame_names[i] << "\n  world: " << contact_pos_world[i].transpose()
              << " body: " << contact_pos_body[i].transpose() << std::endl;
    std::cout << " " << "world: \n"
              << contact_rotation_world[i] << "\n  body: \n"
              << contact_rotation_body[i] << "\n"
              << std::endl;
  }
#if 0
  // clang-format off
  q << pwb, 
       quat.x(), quat.y(), quat.z(), quat.w(),
       1.2, 1.1, -2.2, 1.1, 0, 1, -2, 1;
  // clang-format on
  pinocchio::forwardKinematics(model_fix, data_fix, q.tail(model_fix.nq), qdot.tail(model_fix.nv));
  pinocchio::updateFramePlacements(model_fix, data_fix);
  std::vector<Eigen::Vector3d> contact_pos_body_new;
  contact_pos_body_new.resize(2);

  std::vector<Eigen::Matrix3d> contact_rotation_body_new;
  contact_rotation_body_new.resize(2);
  std::cout << "------ another contact position and rotation ------ " << std::endl;
  for (size_t i = 0; i < contact_frame_names.size(); i++) {
    auto FRAME_ID_fix = model_fix.getFrameId(contact_frame_names[i]);
    contact_pos_body_new[i] =
      data_fix.oMf[FRAME_ID_fix].translation();  // Rbw * (contact_pos_world[i] - pwb);
    contact_rotation_body_new[i] =
      data_fix.oMf[FRAME_ID_fix].rotation();  // Rbw * contact_rotation_world[i];

    std::cout << " " << contact_frame_names[i] << " body: " << contact_pos_body_new[i].transpose()
              << std::endl;
    std::cout << " body: \n" << contact_rotation_body_new[i] << "\n" << std::endl;
    auto pos_err = contact_pos_body_new[i] - contact_pos_body[i];
    auto rot_err = pinocchio::log3(
      contact_rotation_body_new[i] *
      contact_rotation_body[i].transpose());  // pinocchio::log3(iMd.rotation());
    std::cout << " pos_err: " << pos_err.transpose() << std::endl;
    std::cout << " rot_err: " << rot_err.transpose() << std::endl;
  }
#endif
  Eigen::Vector3d pwc, pbc;
  pwc = (contact_pos_world[0] + contact_pos_world[1]) / 2;  // world frame to control frame
  pbc = (contact_pos_body[0] + contact_pos_body[1]) / 2;    // body frame to control frame

  Eigen::Vector3d pcb;
  pcb = -Rbc.transpose() * pbc;
  std::cout << "\n pwc: " << pwc.transpose() << " pbc: " << pbc.transpose()
            << " pcb: " << pcb.transpose() << "\n"
            << std::endl;

  // 构建状态量与关节之间的关系
  Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
  x.head(3) = contact_pos_body[0];
  x.tail(3) = contact_pos_body[1];

  Eigen::MatrixXd s = getSMatrix(Rbc);
  auto y = s * x;  // pcb_x pcb_y pcb_z deltac_x deltac_y deltac_z
  std::cout << " y:     " << y.transpose() << std::endl;
  auto x_inv = s.inverse() * y;
  std::cout << " x:     " << x.transpose() << std::endl;
  std::cout << " x_inv: " << x_inv.transpose() << std::endl;

  // inverse kinematics
  std::cout << "----------------- inverse kinematic ------------------ \n\n" << std::endl;

  Eigen::Vector3d eular_d;
  eular_d <<  0.055, 0.0 / 4, 0.0 / 2;

  Eigen::Matrix3d Rcb_d = eularToQuat(eular_d).toRotationMatrix();  //Rbc.transpose();
  std::cout << " Rcb_d \n" << Rcb_d << "\n " << std::endl;
  Eigen::MatrixXd sd = getSMatrix(Rcb_d.transpose());
  std::cout << " sd \n" << sd << "\n " << std::endl;

  Eigen::VectorXd yd = Eigen::VectorXd::Zero(6);
  yd = y;
  // yd(2) += 0.1;
  Eigen::VectorXd xd = sd.inverse() * yd;
  // xd(1) += 0.1;
  std::cout << " yd:     " << yd.transpose() << std::endl;
  std::cout << " xd:     " << xd.transpose() << std::endl;

  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-0;

  pinocchio::Data::Matrix6x J(6, model_fix.nv);
  J.setZero();

  Eigen::Vector4d err;
  auto start = std::chrono::high_resolution_clock::now();
  Eigen::VectorXd qd = q.tail(model_fix.nv);
  std::cout << " qd_pre: " << qd.transpose() << std::endl;

  for (size_t j = 0; j < contact_frame_names.size(); j++) {
    bool success = false;
    auto FRAME_ID = model.getFrameId(contact_frame_names[j]);

    std::cout << "--------- frame id: " << contact_frame_names[j] << " --------- " << std::endl;
    for (int i = 0;; i++) {
      pinocchio::forwardKinematics(model_fix, data_fix, qd);
      pinocchio::updateFramePlacements(model_fix, data_fix);

      Eigen::Matrix3d rotm = contact_rotation_body[j];
      Eigen::Vector3d trans = xd.segment(3 * j, 3);
      const pinocchio::SE3 oMdes(rotm, trans);
      const pinocchio::SE3 oMnow = data_fix.oMf[FRAME_ID];
      auto pos_err = oMdes.translation() - oMnow.translation();
      auto rot_err = pinocchio::log3(
        oMdes.rotation() * oMnow.rotation().transpose());  // pinocchio::log3(iMd.rotation());

      std::cout << " pos_err: " << pos_err.transpose() << std::endl;
      std::cout << " rot_err: " << rot_err.transpose() << std::endl;

      // std::cout << " oMnow: " << oMnow.translation().transpose() << std::endl;
      // std::cout << " oMnow: " << oMnow.rotation() << std::endl;

      // std::cout << " oMdes: " << oMdes.translation().transpose() << std::endl;
      // std::cout << " oMdes: " << oMdes.rotation() << std::endl;
      std::cout << std::endl;
      pinocchio::computeFrameJacobian(
        model_fix, data_fix, qd, FRAME_ID, pinocchio::LOCAL_WORLD_ALIGNED, J);  // J in joint frame
      // std::cout << " J: \n " << J << "\n " << std::endl;

      Eigen::MatrixXd Jtop(4, 4);
      Jtop.topRows(3) = J.block(0, 4 * j, 3, 4);
      Jtop.bottomRows(1) = J.block(4, 4 * j, 1, 4);

      Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(Jtop.rows(), Jtop.rows());
      auto Jtop_pi =
        Jtop.transpose() * (Jtop * Jtop.transpose() /*+ damp * damp * identity*/).inverse();

      pinocchio::updateFramePlacements(model_fix, data_fix);
      Eigen::Vector4d desire, current;

      err << xd.segment(3 * j, 3) - data_fix.oMf[FRAME_ID].translation(), rot_err(1);
      qd.segment(4 * j, 4) += Jtop_pi * err;

      if (err.norm() < eps) {
        success = true;
        pinocchio::forwardKinematics(model_fix, data_fix, qd);
        xd.segment(j * 3, 3) = data_fix.oMf[FRAME_ID].translation();
        std::cout << "iter times: " << i << std::endl;
        break;
      }
      if (i >= IT_MAX) {
        success = false;
        break;
      }

      // if (!(i % 10)) std::cout << i << ": error = " << err.transpose() << std::endl;
    }
  }
  std::cout << "------ end inverse kinematic ------" << std::endl;
  std::cout << " qd: " << qd.transpose() << std::endl;
  yd = sd * xd;
  std::cout << " xd: " << xd.transpose() << std::endl;
  std::cout << " yd: " << yd.transpose() << std::endl;
  // std::cout << " world: " << std::endl;
  auto Rwb = eularToQuat(eular_d).toRotationMatrix();
  std::cout << " Rwb: " << Rwb << std::endl;
  std::cout << " xd in world: " << (Rwb * xd.segment(0, 3)).transpose() << "  "
            << (Rwb * xd.segment(3, 3)).transpose() << std::endl;
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;
  std::cout << "程序运行时间: " << duration.count() << " 秒" << std::endl;
}