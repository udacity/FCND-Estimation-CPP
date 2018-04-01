#pragma once

#include "SimulatedQuadSensor.h"
#include "QuadDynamics.h"
#include "Math/Random.h"


class SimulatedGPS : public SimulatedQuadSensor
{ 
public:
  SimulatedGPS(string config, string name) : SimulatedQuadSensor(config, name) { Init(); }

  virtual void Init()
  {
    SimulatedQuadSensor::Init();
    ParamsHandle paramSys = SimpleConfig::GetInstance();
    _gpsStd = paramSys->Get(_config + ".Std", V3F(1, 1, 1));
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
    
    // generate error
    V3F error(gasdev_f(idum), gasdev_f(idum), gasdev_f(idum));
    error *= _gpsStd;

    _gpsMeas = quad.Position() + error;
    _freshMeas = true;

    // TODO: velocity!
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
      GETTER_HELPER("GPS.X", _gpsMeas.x);
      GETTER_HELPER("GPS.Y", _gpsMeas.y);
      GETTER_HELPER("GPS.Z", _gpsMeas.z);
#undef GETTER_HELPER
    }
    return false; 
  };

  virtual vector<string> GetFields() const 
  { 
    vector<string> ret = SimulatedQuadSensor::GetFields();
    ret.push_back(_name + ".GPS.x");
    ret.push_back(_name + ".GPS.y");
    ret.push_back(_name + ".GPS.z");
    ret.push_back(_name + ".GPS.vx");
    ret.push_back(_name + ".GPS.vy");
    ret.push_back(_name + ".GPS.vz");
    return ret;
  };

  V3F _gpsMeas;
  V3F _gpsStd;
  float _gpsDT;
};
