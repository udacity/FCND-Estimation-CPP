#include "Common.h"
#include "QuadEKFEstimator.h"
#include "Utility/SimpleConfig.h"

using namespace SLR;

QuadEKFEstimator::QuadEKFEstimator(string config) 
  : BaseQuadEstimator(config) 
{
  Init();
};

void QuadEKFEstimator::Init()
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
}