#include "Common.h"
#include "BaseController.h"
#ifndef __PX4_NUTTX
#include "Utility/SimpleConfig.h"
#endif
#include "Utility/StringUtils.h"
using namespace SLR;

BaseController::BaseController(string name, string config)
{
	_name = name;
  _config = config;
  Init();
}

void BaseController::Init()
{
#ifndef __PX4_NUTTX
  ParamsHandle config = SimpleConfig::GetInstance();

  mass = config->Get(_config+".Mass", 1.f);
  L = config->Get(_config+".L", 0.1f);
  Ixx = config->Get(_config+".Ixx", 0.001f);
  Iyy = config->Get(_config + ".Iyy", 0.001f);
  Izz = config->Get(_config + ".Izz", 0.002f);
  kappa = config->Get(_config + ".kappa", 0.01f);

  trajectory.Clear();
  string trajFile = config->Get(_config + ".Trajectory","");
  if (!trajectory.ReadFile(string("../config/")+trajFile))
  {
    TrajectoryPoint tmp;
    tmp.position = config->Get(_config + ".Trajectory", V3F());
    trajectory.AddTrajectoryPoint(tmp);
  }
#else

#endif
}

void BaseController::Reset()
{
  // Reinitialise the physical parameters
  Init();
}

void BaseController::UpdateEstimates(V3F pos, V3F vel, Quaternion<float> attitude, V3F omega)
{
  estAtt = attitude;
  estOmega = omega;
  estPos = pos;
  estVel = vel;
}

TrajectoryPoint BaseController::GetNextTrajectoryPoint(float mission_time)
{
  TrajectoryPoint pt = trajectory.NextTrajectoryPoint(mission_time + _trajectoryTimeOffset);
  pt.position += _trajectoryOffset;
  return pt;  
}

// Access functions for graphing variables
bool BaseController::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');

  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    // UDACITY CONVENTION
    GETTER_HELPER("Ref.X", curTrajPoint.position.x);
    GETTER_HELPER("Ref.Y", curTrajPoint.position.y);
    GETTER_HELPER("Ref.Z", curTrajPoint.position.z);
		GETTER_HELPER("Ref.VX", curTrajPoint.velocity.x);
		GETTER_HELPER("Ref.VY", curTrajPoint.velocity.y);
		GETTER_HELPER("Ref.VZ", curTrajPoint.velocity.z);
		GETTER_HELPER("Ref.Yaw", curTrajPoint.attitude.Yaw());
#undef GETTER_HELPER
  }
  return false;
}

vector<string> BaseController::GetFields() const
{
  vector<string> ret;
  ret.push_back(_name + ".Ref.X");
  ret.push_back(_name + ".Ref.Y");
  ret.push_back(_name + ".Ref.Z");
	ret.push_back(_name + ".Ref.VX");
	ret.push_back(_name + ".Ref.VY");
	ret.push_back(_name + ".Ref.VZ");
	ret.push_back(_name + ".Ref.Yaw");
  return ret;
}
