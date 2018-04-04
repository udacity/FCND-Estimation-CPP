#pragma once

#include "BaseQuadEstimator.h"
#include "matrix/math.hpp"

using matrix::Vector;
using matrix::Matrix;
using matrix::SquareMatrix;

class QuadEstimatorEKF : public BaseQuadEstimator
{
public:
  QuadEstimatorEKF(string config, string name);

  virtual void Init();

  virtual void Predict(float dt, V3F accel, V3F gyro);
  virtual void UpdateFromIMU(V3F accel, V3F gyro);
  virtual void UpdateFromGPS(V3F pos, V3F vel);
  virtual void UpdateFromBaro(float z) {};
  virtual void UpdateFromMag(V3F mag) {};

  static const int QUAD_EKF_NUM_STATES = 7;

  // process covariance
  SquareMatrix<float, QUAD_EKF_NUM_STATES> Q; 

	// GPS covariance
	Matrix<float, 6, 6> R_GPS;

  // attitude filter state
  float pitchEst, rollEst;
  float accelPitch, accelRoll; // raw pitch/roll angles as calculated from last accelerometer.. purely for graphing.
	V3F accelG;

  // generic update
  template<size_t numZ> 
  void Update(matrix::Vector<float, numZ>& z,
    matrix::Matrix<float, numZ, QUAD_EKF_NUM_STATES>& H,
    matrix::Matrix<float, numZ, numZ>& R,
    matrix::Vector<float, numZ>& hOfU);

  // EKF state  
  matrix::Vector<float, QUAD_EKF_NUM_STATES> state;
  matrix::SquareMatrix<float, QUAD_EKF_NUM_STATES> cov;

  // params
  float attitudeTau;
  float dtIMU;

  // Access functions for graphing variables
  virtual bool GetData(const string& name, float& ret) const;
  virtual vector<string> GetFields() const;
  string _name;

};