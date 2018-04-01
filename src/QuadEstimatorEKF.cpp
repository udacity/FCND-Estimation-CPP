#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"

using namespace SLR;

QuadEstimatorEKF::QuadEstimatorEKF(string config, string name) 
  : BaseQuadEstimator(config) 
{
  _name = name;
  Init();
};

void QuadEstimatorEKF::Init()
{
  ParamsHandle paramSys = SimpleConfig::GetInstance();

  paramSys->GetFloatVector(_config + ".InitState", state);
  
  matrix::Vector<float, QUAD_EKF_NUM_STATES> initStdDevs;
  paramSys->GetFloatVector(_config + ".InitStdDevs", initStdDevs);
  cov.setIdentity();
  for (int i = 0; i < QUAD_EKF_NUM_STATES; i++)
  {
    cov(i, i) = initStdDevs(i) * initStdDevs(i);
  }

  // complementary filter params
  attitudeTau = paramSys->Get(_config + ".AttitudeTau", .1f);
  dtIMU = paramSys->Get(_config + ".dtIMU", .001f);

  pitchEst = 0;
  rollEst = 0;

  // TODO: load measurement cov
}

void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
  float pitchMeas = atan2f(-accel.x, accel.z);
  pitchEst = attitudeTau / (attitudeTau + dtIMU) * (pitchEst + dtIMU * gyro.y) + dtIMU / (attitudeTau + dtIMU) * pitchMeas;

  float rollMeas = atan2f(accel.y, accel.z);
  rollEst = attitudeTau / (attitudeTau + dtIMU) * (rollEst + dtIMU * gyro.x) + dtIMU / (attitudeTau + dtIMU) * rollMeas;

}

// Access functions for graphing variables
bool QuadEstimatorEKF::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');

  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    GETTER_HELPER("Est.roll", rollEst);
    GETTER_HELPER("Est.pitch", pitchEst);
#undef GETTER_HELPER
  }
  return false;
};

vector<string> QuadEstimatorEKF::GetFields() const
{
  vector<string> ret = BaseQuadEstimator::GetFields();
  ret.push_back(_name + ".Est.roll");
  ret.push_back(_name + ".Est.pitch");
  return ret;
};
