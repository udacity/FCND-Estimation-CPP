#pragma once

#include "BaseQuadEstimator.h"
#include "matrix/math.hpp"


class QuadEKFEstimator : public BaseQuadEstimator
{
public:
  QuadEKFEstimator(string config);

  virtual void Init();

  virtual void Predict(float dt, V3F gyro, V3F accel) {};

  virtual void UpdateFromIMU(V3F accel, V3F gyro, V3F accelStd, V3F gyroStd) {};
  virtual void UpdateFromGPS(V3F gpsNED, V3F gpsStd) {};
  virtual void UpdateFromBaro(float z, float std) {};
  virtual void UpdateFromMag(V3F mag, V3F magStd) {};

  // attitude filter state
  float pitch, roll;

  // EKF state  
  static const int QUAD_EKF_NUM_STATES = 7;
  matrix::Vector<float, QUAD_EKF_NUM_STATES> state;
  matrix::Matrix<float, QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES> cov;
};