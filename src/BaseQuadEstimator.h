#pragma once

#include "Math/V3F.h"

class BaseQuadEstimator
{
public:
  BaseQuadEstimator(string config);

  virtual void Init() {};

  virtual void Predict(float dt, V3F gyro, V3F accel) {};
  
  virtual void UpdateFromIMU(V3F accel, V3F gyro, V3F accelStd, V3F gyroStd) {};
  virtual void UpdateFromGPS(V3F gpsNED, V3F gpsStd) {};
  virtual void UpdateFromBaro(float z, float std) {};
  virtual void UpdateFromMag(V3F mag, V3F magStd) {};
  virtual void UpdateFromOpticalFlow(float dx, float dy, float std) {};
  virtual void UpdateFromRangeSensor(float rng, float std) {};

  string _config;
};