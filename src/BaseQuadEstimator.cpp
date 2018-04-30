#include "Common.h"
#include "BaseQuadEstimator.h"

BaseQuadEstimator::BaseQuadEstimator(string config)
{
  _config = config;
  Init();
}

BaseQuadEstimator::~BaseQuadEstimator()
{

}
