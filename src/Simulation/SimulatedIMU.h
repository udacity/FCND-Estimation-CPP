#pragma once

#include "SimulatedQuadSensor.h"
#include "QuadDynamics.h"
#include "Math/Random.h"
#include "BaseQuadEstimator.h"

class SimulatedIMU : public SimulatedQuadSensor
{ 
public:
  SimulatedIMU(string config, string name) : SimulatedQuadSensor(config, name) { Init(); }

  virtual void Init()
  {
    SimulatedQuadSensor::Init();
    ParamsHandle paramSys = SimpleConfig::GetInstance();
    _accelStd = paramSys->Get(_config + ".AccelStd", V3F());
    _gyroStd = paramSys->Get(_config + ".GyroStd", V3F());
    _gpsDT = paramSys->Get(_config + ".dt", .1f);
  }

  // if it's time, generates a new sensor measurement, saves it internally (for graphing), and calls appropriate estimator update function
  virtual void Update(QuadDynamics& quad, shared_ptr<BaseQuadEstimator> estimator, float dt, int& idum)
  {
    _timeAccum += dt;
    if (_timeAccum < _gpsDT)
    {
      return;
    }

    _timeAccum = (_timeAccum - _gpsDT);
    
    // accelerometer
    V3F accelError = V3F(gasdev_f(idum), gasdev_f(idum), gasdev_f(idum)) * _accelStd;
    _accelMeas = quad.Attitude().Rotate_ItoB(quad.Acceleration() + V3F(0,0,9.81f)) + accelError;
    _accelMeas.constrain(-6.f*9.81f, 6.f*9.81f);

    // rate gyro
    V3F gyroError = V3F(gasdev_f(idum), gasdev_f(idum), gasdev_f(idum)) * _gyroStd;
    _gyroMeas = quad.Omega() + gyroError;

    _freshMeas = true;

    if (estimator)
    {
			// TODO: update happens before prediction because predict uses the attitude and update
			// updates the attitude...
			estimator->UpdateFromIMU(_accelMeas, _gyroMeas);
      estimator->Predict(dt, _accelMeas, _gyroMeas);
    }
  };

  // Access functions for graphing variables
  // note that GetData will only return true if a fresh measurement was generated last Update()
  virtual bool GetData(const string& name, float& ret) const 
  {
    if (!_freshMeas) return false;

    if (name.find_first_of(".") == string::npos) return false;
    string leftPart = LeftOf(name, '.');
    string rightPart = RightOf(name, '.');

    if (ToUpper(leftPart) == ToUpper(_name))
    {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
      GETTER_HELPER("IMU.AX", _accelMeas.x);
      GETTER_HELPER("IMU.AY", _accelMeas.y);
      GETTER_HELPER("IMU.AZ", _accelMeas.z);
      GETTER_HELPER("IMU.GX", _gyroMeas.x);
      GETTER_HELPER("IMU.GY", _gyroMeas.y);
      GETTER_HELPER("IMU.GZ", _gyroMeas.z);
#undef GETTER_HELPER
    }
    return false; 
  };

  virtual vector<string> GetFields() const 
  { 
    vector<string> ret = SimulatedQuadSensor::GetFields();
    ret.push_back(_name + ".IMU.ax");
    ret.push_back(_name + ".IMU.ay");
    ret.push_back(_name + ".IMU.az");
    ret.push_back(_name + ".IMU.gx");
    ret.push_back(_name + ".IMU.gy");
    ret.push_back(_name + ".IMU.gz");
    return ret;
  };

  V3F _accelMeas, _gyroMeas;
  V3F _accelStd, _gyroStd;
  float _gpsDT;
};
