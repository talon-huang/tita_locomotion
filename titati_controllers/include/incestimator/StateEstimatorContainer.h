/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

// #include "ControlParameters/RobotParameters.h"
// #include "Controllers/LegController.h"
#include "common/IMUTypes.h"
// #include "SimUtilities/VisualizationData.h"
// #include "state_estimator_lcmt.hpp"

/*!
 * Result of state estimation
 */
template <typename T>
struct StateEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;
  Vec3<T> position;
  Vec3<T> vBody;
  Quat<T> orientation;
  Vec3<T> omegaBody;
  RotMat<T> rBody;
  Vec3<T> rpy;

  Vec3<T> omegaWorld;
  Vec3<T> vWorld;
  Vec3<T> aBody, aWorld;

  Vec3< T > terrainCoefficient;
  Mat3< T > terrainRotationMatrix;

  // void setLcm(state_estimator_lcmt& lcm_data) {
  //   for(int i = 0; i < 3; i++) {
  //     lcm_data.p[i] = position[i];
  //     lcm_data.vWorld[i] = vWorld[i];
  //     lcm_data.vBody[i] = vBody[i];
  //     lcm_data.rpy[i] = rpy[i];
  //     lcm_data.omegaBody[i] = omegaBody[i];
  //     lcm_data.omegaWorld[i] = omegaWorld[i];
  //   }

  //   for(int i = 0; i < 4; i++) {
  //     lcm_data.quat[i] = orientation[i];
  //   }
  // }
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
template <typename T>
struct StateEstimatorData
{
  StateEstimate<T>* result; // where to write the output to
  // VectorNavData vectorNavData;
  // CheaterState<double> *cheaterState;
  // LegControllerData<T> *legControllerData;
  // Vec4<T> contactPhase;
  // RobotControlParameters *parameters;
};

/*!
 * All Estimators should inherit from this class
 */
template <typename T>
class GenericEstimator
{
public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimate<T>* data) { _stateEstimatorData.result = data;}

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
};

#endif // PROJECT_STATEESTIMATOR_H
