#pragma once

#include "BaseQuadEstimator.h"
#include "matrix/math.hpp"


class QuadEstimatorEKF : public BaseQuadEstimator
{
public:
  QuadEstimatorEKF(string config, string name);

  virtual void Init();

  virtual void Predict(float dt, V3F gyro, V3F accel) {};

  virtual void UpdateFromIMU(V3F accel, V3F gyro);
  virtual void UpdateFromGPS(V3F pos, V3F vel) {};
  virtual void UpdateFromBaro(float z) {};
  virtual void UpdateFromMag(V3F mag) {};

  // attitude filter state
  float pitchEst, rollEst;

  // EKF state  
  static const int QUAD_EKF_NUM_STATES = 7;
  matrix::Vector<float, QUAD_EKF_NUM_STATES> state;
  matrix::Matrix<float, QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES> cov;

  // params
  float attitudeTau;
  float dtIMU;

  // Access functions for graphing variables
  virtual bool GetData(const string& name, float& ret) const;
  virtual vector<string> GetFields() const;
  string _name;

};