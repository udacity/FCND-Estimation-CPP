#pragma once

#include "SimulatedQuadSensor.h"
#include "QuadDynamics.h"
#include "Math/Random.h"
#include "BaseQuadEstimator.h"

class SimulatedGPS : public SimulatedQuadSensor
{ 
public:
  SimulatedGPS(string config, string name) : SimulatedQuadSensor(config, name) { Init(); }

  virtual void Init()
  {
    SimulatedQuadSensor::Init();
    ParamsHandle paramSys = SimpleConfig::GetInstance();
    _posStd = paramSys->Get(_config + ".PosStd", V3F());
    _posRandomWalkStd = paramSys->Get(_config + ".PosRandomWalkStd", V3F());
    _velStd = paramSys->Get(_config + ".VelStd", V3F());
    _gpsDT = paramSys->Get(_config + ".dt", .1f);
    
    _posRandomWalk = V3F();
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
    
    // position
    _posRandomWalk += V3F(gasdev_f(idum), gasdev_f(idum), gasdev_f(idum)) * _posRandomWalkStd;
    V3F posError = V3F(gasdev_f(idum), gasdev_f(idum), gasdev_f(idum)) * _posStd + _posRandomWalk;
    _posMeas = quad.Position() + posError;

    // velocity
    V3F velError = V3F(gasdev_f(idum), gasdev_f(idum), gasdev_f(idum)) * _velStd;
    _velMeas = quad.Velocity() + velError;

    _freshMeas = true;

    if (estimator)
    {
      estimator->UpdateFromGPS(_posMeas, _velMeas);
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
      GETTER_HELPER("GPS.X", _posMeas.x);
      GETTER_HELPER("GPS.Y", _posMeas.y);
      GETTER_HELPER("GPS.Z", _posMeas.z);
      GETTER_HELPER("GPS.VX", _velMeas.x);
      GETTER_HELPER("GPS.VY", _velMeas.y);
      GETTER_HELPER("GPS.VZ", _velMeas.z);
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

  V3F _posMeas, _velMeas;
  V3F _posRandomWalk;
  V3F _posStd, _velStd;
  V3F _posRandomWalkStd;
  float _gpsDT;
};
