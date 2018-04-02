#pragma once

class BaseQuadEstimator;

class SimulatedQuadSensor : public DataSource
{
public:
  SimulatedQuadSensor(string config, string name) : _config(config), _name(name) { Init(); }

  virtual void Init() 
  {
    _freshMeas = false;
    _timeAccum = 0;
  };
  
  // if it's time, generates a new sensor measurement, saves it internally (for graphing), and calls appropriate estimator update function
  virtual void Update(QuadDynamics& quadDynamics, shared_ptr<BaseQuadEstimator> estimator, float dt, int& idum) {};

  // Access functions for graphing variables
  // note that GetData will only return true if a fresh measurement was generated last Update()
  virtual bool GetData(const string& name, float& ret) const { return false; };
  virtual vector<string> GetFields() const { return vector<string>(); };
  virtual void FinalizeDataFrame() { _freshMeas = false; }

  string _config, _name;
  bool _freshMeas;
  float _timeAccum;
};

