/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "incdynamics/FloatingBaseModel.h"
#include "incdynamics/Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  cheetah._bodyMass = 30;
  cheetah._bodyLength = 0.26 * 2;
  cheetah._bodyWidth = 0.0895 * 2;
  cheetah._bodyHeight = 0.05 * 2;
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio = 6;
  cheetah._kneeGearRatio = 9.33;
  cheetah._abadLinkLength = 0.1413;
  cheetah._hipLinkLength = 0.2;
  //cheetah._kneeLinkLength = 0.175;
  //cheetah._maxLegLength = 0.384;
  cheetah._kneeLinkY_offset = 0.00;
  //cheetah._kneeLinkLength = 0.20;
  cheetah._kneeLinkLength = 0.2;
  cheetah._maxLegLength = 0.409;


  cheetah._motorTauMax = 3.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 0e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 1998, 0, 0, 0, 2331, 0, 0, 0, 3915;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0.00099, 0.000264, 0.007126);  // LEFT
  SpatialInertia<T> abadInertia(1.8218, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 5688, 0, 0, 0, 7626, 0, 0, 0, 2311;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0.0001, 0.0285, -0.0225);
  SpatialInertia<T> hipInertia(2.2809, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 2492, 0, 0, 0, 2658, 0, 0, 0, 229;
  kneeRotationalInertiaRotated = 3*kneeRotationalInertiaRotated * 1e-6;
//   kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(-0.0085, 0.0065, -0.1043);
  SpatialInertia<T> kneeInertia(3*0.5667, kneeCOM, kneeRotationalInertiaRotated);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0*0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0*0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 2*49519, 0, 0, 0, 6*65137, 0, 0, 0, 6*91429;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
