#pragma once

#include "DataSource.h"
#include "Math/V3F.h"
#include "Math/Quaternion.h"

using SLR::Quaternion;

class BaseQuadEstimator : public DataSource
{
public:
  BaseQuadEstimator(string config);
  virtual ~BaseQuadEstimator();

  virtual void Init() {};

  virtual void Predict(float dt, V3F accel, V3F gyro) {};
  
  virtual void UpdateFromIMU(V3F accel, V3F gyro) {};
  virtual void UpdateFromGPS(V3F pos, V3F vel) {};
  virtual void UpdateFromBaro(float z) {};
  virtual void UpdateFromMag(float magYaw) {};
  virtual void UpdateFromOpticalFlow(float dx, float dy) {};
  virtual void UpdateFromRangeSensor(float rng) {};

	virtual void UpdateTrueError(V3F truePos, V3F trueVel, Quaternion<float> trueAtt) {};

	virtual V3F EstimatedPosition() = 0;
	virtual V3F EstimatedVelocity()=0;
	virtual Quaternion<float> EstimatedAttitude()=0;
	virtual V3F EstimatedOmega()=0;

  string _config;
};
