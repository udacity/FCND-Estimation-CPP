#pragma once

#include "SimulatedQuadSensor.h"
#include "QuadDynamics.h"
#include "Math/Random.h"
#include "BaseQuadEstimator.h"

class SimulatedMag : public SimulatedQuadSensor
{ 
public:
  SimulatedMag(string config, string name) : SimulatedQuadSensor(config, name) { Init(); }

  virtual void Init()
  {
    SimulatedQuadSensor::Init();
    ParamsHandle paramSys = SimpleConfig::GetInstance();
    _magStd = paramSys->Get(_config + ".Std", 0);
		_measDT = paramSys->Get(_config + ".dt", .01f);
		_magYaw = 0;
  }

  // if it's time, generates a new sensor measurement, saves it internally (for graphing), and calls appropriate estimator update function
  virtual void Update(QuadDynamics& quad, shared_ptr<BaseQuadEstimator> estimator, float dt, int& idum)
  {
    _timeAccum += dt;
    if (_timeAccum < _measDT)
    {
      return;
    }

    _timeAccum = (_timeAccum - _measDT);
    
    // position
    float magError = gasdev_f(idum) * _magStd;
    _magYaw = quad.Attitude().Yaw() + magError;
		if (_magYaw > F_PI) _magYaw -= 2.f*F_PI;
		if (_magYaw < -F_PI) _magYaw += 2.f*F_PI;

    _freshMeas = true;

    if (estimator)
    {
      estimator->UpdateFromMag(_magYaw);
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
      GETTER_HELPER("magYaw", _magYaw);
#undef GETTER_HELPER
    }
    return false; 
  };

  virtual vector<string> GetFields() const 
  { 
    vector<string> ret = SimulatedQuadSensor::GetFields();
    ret.push_back(_name + ".MagYaw");
    return ret;
  };

	float _magYaw;	// last yaw measurement from magnetometer
	float _magStd;	// std deviation of noise for magnetometer measurements
  float _measDT;		// time (in seconds) between measurements
};
