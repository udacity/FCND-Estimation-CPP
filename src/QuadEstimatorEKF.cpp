#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Math/Quaternion.h"

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
	// FUTURE NOTE TO STUDENT:
	// You can try using a small-angle approximation (treat pitch and roll as completely separate) but 
	// for any interesting "dynamic" trajectories this can break down quickly. Try doing a more proper
	// gyro integration based on the rotation matrix instead

	// SMALL ANGLE GYRO INTEGRATION:
	//float predictedPitch = pitchEst + dtIMU * gyro.y;
	//float predictedRoll = rollEst + dtIMU * gyro.x;

	// BETTER INTEGRATION:
	Quaternion<float> quat = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, state(6));
	quat.IntegrateBodyRate(gyro, dtIMU);
	float predictedPitch = quat.Pitch();
	float predictedRoll = quat.Roll();

	// CALCULATE UPDATE
	accelRoll = atan2f(accel.y, accel.z);
	accelPitch = atan2f(-accel.x, 9.81f);

	// FUSE INTEGRATION AND UPDATE
	rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll) + dtIMU / (attitudeTau + dtIMU) * accelRoll;
  pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch) + dtIMU / (attitudeTau + dtIMU) * accelPitch;

	// YAW -- TODO - weird that it's here..
	state(6) = quat.Yaw();
}

void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
  matrix::Vector<float, QUAD_EKF_NUM_STATES> newState = state;

  // note attitude pitch/roll is already done in "UpdateFromIMU"

  // integrate positions
  newState(0) += state(3)*dt;
  newState(1) += state(4)*dt;
  newState(2) += state(5)*dt;

  // integrate velocity
  Quaternion<float> att = SLR::Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, state(6));
  accelG = att.Rotate_BtoI(accel) - V3F(0, 0, 9.81f);
  newState(3) += accelG[0] * dt;
  newState(4) += accelG[1] * dt;
  newState(5) += accelG[2] * dt;

  // yaw - done in UpdateFromIMU
  //newState(6) += att.Rotate_BtoI(gyro).z * dt;

  // TRANSITION MATRIX JACOBIAN

  // first, figure out the Rbg_prime
  // TODO: CHECK!
  matrix::Matrix<float, 3, 3> Rbg_prime;
  float sinPhi = sin(rollEst), cosPhi = cos(rollEst);
  float sinTheta = sin(pitchEst), cosTheta = cos(pitchEst);
  float sinPsi=sin(state(6)), cosPsi = cos(state(6)); // yaw
  Rbg_prime.setZero();
  Rbg_prime(0, 0) = -cosPhi*sinPsi - sinPhi*cosTheta*cosPsi;
  Rbg_prime(0, 1) = cosPhi*cosPsi - sinPhi*cosTheta*sinPsi;
  Rbg_prime(1, 0) = sinPhi*sinPsi - cosPhi*cosTheta*cosPsi;
  Rbg_prime(1, 1) = -sinPhi*cosPsi - cosPhi*cosTheta*sinPsi;
  Rbg_prime(2, 0) = sinTheta*cosPsi;
  Rbg_prime(2, 1) = sinTheta*sinPsi;

  matrix::Vector<float, 3> u03dt;
  u03dt(0) = accel[0] * dt;
  u03dt(1) = accel[1] * dt;
  u03dt(2) = accel[2] * dt;

  matrix::Vector<float, 3> Rbg_prime_times_u03_dt = Rbg_prime*u03dt;
  
  matrix::SquareMatrix<float, QUAD_EKF_NUM_STATES> gPrime;
  gPrime.setIdentity();
  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = Rbg_prime_times_u03_dt(0);
  gPrime(4, 6) = Rbg_prime_times_u03_dt(1);
  gPrime(5, 6) = Rbg_prime_times_u03_dt(2);

  cov = gPrime * cov * gPrime.T() + Q();
	state = newState;
}

void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  matrix::Vector<float, 6> z, hOfU;
  z(0) = pos.x;
  z(1) = pos.y;
  z(2) = pos.z;
  z(3) = vel.x;
  z(4) = vel.y;
  z(5) = vel.z;

  matrix::Matrix<float, 6, 6> R;
  // TODO: param!
  R.setIdentity();
  R(0, 0) = R(1, 1) = 1;
  R(2, 2) = 3;
  R(3, 3) = R(4, 4) = .1f;
  R(5, 5) = .3f;

  for (int i = 0; i < 6; i++)
  {
    hOfU(i) = state(i);
  }

  matrix::Matrix<float, 6, 7> hPrime;
  hPrime.setIdentity();

  Update(z, hPrime, R, hOfU);
}

template<size_t numZ>
void QuadEstimatorEKF::Update(matrix::Vector<float, numZ>& z,
  matrix::Matrix<float, numZ, QUAD_EKF_NUM_STATES>& H,
  matrix::Matrix<float, numZ, numZ>& R,
  matrix::Vector<float, numZ>& hOfU)
{
  matrix::SquareMatrix<float, numZ> toInvert;
  toInvert = H*cov*H.T() + R;
  matrix::Matrix<float, QUAD_EKF_NUM_STATES,numZ> K = cov * H.T() * inv(toInvert);
  state = state + K*(z - hOfU);

  matrix::SquareMatrix<float, QUAD_EKF_NUM_STATES> eye;
  eye.setIdentity();
  cov = (eye - K*H)*cov;
}

matrix::SquareMatrix<float, QuadEstimatorEKF::QUAD_EKF_NUM_STATES> QuadEstimatorEKF::Q()
{
  // TODO
  matrix::SquareMatrix<float, QuadEstimatorEKF::QUAD_EKF_NUM_STATES> ret;
  ret.setIdentity();
  ret(0, 0) = .1f;
  ret(1, 1) = .1f;
  ret(2, 2) = .1f;
  ret(3, 3) = .1f;
  ret(4, 4) = .1f;
  ret(5, 5) = .1f;
  ret(6, 6) = .001f;
  return ret;
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

    GETTER_HELPER("Est.x", state(0));
    GETTER_HELPER("Est.y", state(1));
    GETTER_HELPER("Est.z", state(2));
    GETTER_HELPER("Est.vx", state(3));
    GETTER_HELPER("Est.vy", state(4));
    GETTER_HELPER("Est.vz", state(5));
    GETTER_HELPER("Est.yaw", state(6));
    
    GETTER_HELPER("Est.S.x", sqrtf(cov(0,0)));
    GETTER_HELPER("Est.S.y", sqrtf(cov(1, 1)));
    GETTER_HELPER("Est.S.z", sqrtf(cov(2, 2)));
    GETTER_HELPER("Est.S.vx", sqrtf(cov(3, 3)));
    GETTER_HELPER("Est.S.vy", sqrtf(cov(4, 4)));
    GETTER_HELPER("Est.S.vz", sqrtf(cov(5, 5)));
    GETTER_HELPER("Est.S.yaw", sqrtf(cov(6, 6)));

    // diagnostic variables
    GETTER_HELPER("Est.D.AccelPitch", accelPitch);
    GETTER_HELPER("Est.D.AccelRoll", accelRoll);

		GETTER_HELPER("Est.D.ax_g", accelG[0]);
		GETTER_HELPER("Est.D.ay_g", accelG[1]);
		GETTER_HELPER("Est.D.az_g", accelG[2]);
#undef GETTER_HELPER
  }
  return false;
};

vector<string> QuadEstimatorEKF::GetFields() const
{
  vector<string> ret = BaseQuadEstimator::GetFields();
  ret.push_back(_name + ".Est.roll");
  ret.push_back(_name + ".Est.pitch");

  ret.push_back(_name + ".Est.x");
  ret.push_back(_name + ".Est.y");
  ret.push_back(_name + ".Est.z");
  ret.push_back(_name + ".Est.vx");
  ret.push_back(_name + ".Est.vy");
  ret.push_back(_name + ".Est.vz");
	ret.push_back(_name + ".Est.yaw");

  ret.push_back(_name + ".Est.S.x");
  ret.push_back(_name + ".Est.S.y");
  ret.push_back(_name + ".Est.S.z");
  ret.push_back(_name + ".Est.S.vx");
  ret.push_back(_name + ".Est.S.vy");
  ret.push_back(_name + ".Est.S.vz");
  ret.push_back(_name + ".Est.S.yaw");

  // diagnostic variables
  ret.push_back(_name + ".Est.D.AccelPitch");
  ret.push_back(_name + ".Est.D.AccelRoll");
	ret.push_back(_name + ".Est.D.ax_g");
	ret.push_back(_name + ".Est.D.ay_g");
	ret.push_back(_name + ".Est.D.az_g");
  return ret;
};
