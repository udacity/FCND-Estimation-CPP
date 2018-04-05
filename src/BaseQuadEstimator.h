#pragma once

#include "DataSource.h"
#include "Math/V3F.h"

class BaseQuadEstimator : public DataSource
{
public:
  BaseQuadEstimator(string config);

  virtual void Init() {};

  virtual void Predict(float dt, V3F accel, V3F gyro) {};
  
  virtual void UpdateFromIMU(V3F accel, V3F gyro) {};
  virtual void UpdateFromGPS(V3F pos, V3F vel) {};
  virtual void UpdateFromBaro(float z) {};
  virtual void UpdateFromMag(float magYaw) {};
  virtual void UpdateFromOpticalFlow(float dx, float dy) {};
  virtual void UpdateFromRangeSensor(float rng) {};

  string _config;
};