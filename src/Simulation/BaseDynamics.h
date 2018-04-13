#pragma once

#include "Common.h"
#include "Math/Quaternion.h"
#include "VehicleDatatypes.h"
#include "DataSource.h"
#include "Utility/FixedQueue.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4100) // unreferenced formal parameter
#endif

using SLR::Quaternion;
class Trajectory;

class BaseDynamics : public DataSource
{
public:
	BaseDynamics(string name="");
	virtual ~BaseDynamics() {}; // destructor

	virtual int Initialize();

  virtual void Run(float dt, float simulationTime, int &idum,  // updates the simulation
    V3F externalForceInGlobalFrame = V3F(),    // required to take net forces into account
    V3F externalMomentInBodyFrame = V3F())   // required to take net moments into account
  {}

	virtual void SetCommands(const VehicleCommand& cmd) {};	// update commands in the simulator coming from a command2 packet

	// inheritors have no reason to alter the following functions and therefore no sense demanding that they do
	GlobalPose     GenerateGP () ; // returns the current simulation state in Vicon format - const?

  V3F Position() const { return pos; };
  V3F Velocity() const { return vel; };
  V3F Acceleration() const { return acc; };
  V3F Omega() const { return omega; };
  Quaternion<float> Attitude() const { return quat; }

  void SetPosition(const V3F& p) { pos = p; }
  void SetVelocity(const V3F& v) { vel = v; }
  void SetOmega(const V3F& o) { omega = o; }
	void SetAttitude(const Quaternion<float>& q){quat = q;}

  virtual bool GetData(const string& name, float& ret) const;
  virtual vector<string> GetFields() const;  

	virtual double GetRotDistInt() { return 0;};
	virtual double GetXyzDistInt() {return 0;};
	virtual double GetRotDistBW() {return 0;};
	virtual double GetXyzDistBW() {return 0;};
	virtual double GetGyroNoiseInt() {return 0;};

  void ResetState(V3F pos=V3F(), V3F vel=V3F(), Quaternion<float> att=Quaternion<float>(), V3F omega=V3F());

	FixedQueue<V3F> _followedPos;
	FixedQueue<Quaternion<float>> _followedAtt;

protected:
  string _name;

	// pos, vel, acc are in the global frame while omega is in the local frame (body rates)
	V3F  pos, vel, acc, omega, old_omega; 
  Quaternion<float>  quat;

  // vehicle geometry and mass properties
  float M; // veh mass, kg
  float Ixx,Iyy,Izz;
  float xMin,yMin,zMin,xMax,yMax,zMax;

  float _lastTrajPointTime;
  float _trajLogStepTime;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif
