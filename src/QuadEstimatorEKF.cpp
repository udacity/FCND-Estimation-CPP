#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Math/Quaternion.h"

#include "Eigen/Dense"
#include "Eigen/SVD"
using Eigen::MatrixXf;
using Eigen::VectorXf;

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
  
	VectorXf initStdDevs(QUAD_EKF_NUM_STATES);
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

	R_GPS.setZero();
	R_GPS(0, 0) = R_GPS(1, 1) = powf(paramSys->Get(_config + ".GPSPosXYStd", 0), 2);
	R_GPS(2, 2) = powf(paramSys->Get(_config + ".GPSPosZStd", 0), 2);
	R_GPS(3, 3) = R_GPS(4, 4) = powf(paramSys->Get(_config + ".GPSVelXYStd", 0), 2);
	R_GPS(5, 5) = powf(paramSys->Get(_config + ".GPSVelZStd", 0), 2);

	R_Yaw.setZero();
	R_Yaw(0,0) = powf(paramSys->Get(_config + ".MagYawStd", 0), 2);

	Q.setZero();
	Q(0, 0) = Q(1, 1) = powf(paramSys->Get(_config + ".QPosXYStd", 0), 2);
	Q(2, 2) = powf(paramSys->Get(_config + ".QPosZStd", 0), 2);
	Q(3, 3) = Q(4, 4) = powf(paramSys->Get(_config + ".QVelXYStd", 0), 2);
	Q(5, 5) = powf(paramSys->Get(_config + ".QVelZStd", 0), 2);
	Q(6, 6) = powf(paramSys->Get(_config + ".QYawStd", 0), 2);
	Q *= dtIMU;

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

	lastGyro = gyro;
}

void QuadEstimatorEKF::UpdateTrueError(V3F truePos, V3F trueVel, Quaternion<float> trueAtt)
{
	Vector<float, 7> trueState;
	trueState(0) = truePos.x;
	trueState(1) = truePos.y;
	trueState(2) = truePos.z;
	trueState(3) = trueVel.x;
	trueState(4) = trueVel.y;
	trueState(5) = trueVel.z;
	trueState(6) = trueAtt.Yaw();

	trueError = state - trueState;
	if (trueError(6) > F_PI) trueError(6) -= 2.f*F_PI;
	if (trueError(6) < -F_PI) trueError(6) += 2.f*F_PI;

	pitchErr = pitchEst - trueAtt.Pitch();
	rollErr = rollEst - trueAtt.Roll();
}


void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
  Vector<float, QUAD_EKF_NUM_STATES> newState = state;

  // note attitude pitch/roll is already done in "UpdateFromIMU"

  // integrate positions
  newState(0) += state(3)*dt;
  newState(1) += state(4)*dt;
  newState(2) += state(5)*dt;

  // integrate velocity
  Quaternion<float> att = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, state(6));
  accelG = att.Rotate_BtoI(accel) - V3F(0, 0, 9.81f);
  newState(3) += accelG[0] * dt;
  newState(4) += accelG[1] * dt;
  newState(5) += accelG[2] * dt;

  // yaw - done in UpdateFromIMU
  //newState(6) += att.Rotate_BtoI(gyro).z * dt;

  // TRANSITION MATRIX JACOBIAN

  // first, figure out the Rbg_prime
  // TODO: CHECK!
  Matrix<float, 3, 3> Rbg_prime;
  float sinPhi =		sin(rollEst),		cosPhi = cos(rollEst);
  float sinTheta =	sin(pitchEst),	cosTheta = cos(pitchEst);
  float sinPsi=			sin(state(6)),	cosPsi = cos(state(6)); // yaw
  Rbg_prime.setZero();
	// Diebel eq 53 (Euler 3,1,3 -- I think we're 1,2,3!)
  /*Rbg_prime(0, 0) = -cosPhi*sinPsi - sinPhi*cosTheta*cosPsi;
  Rbg_prime(0, 1) = cosPhi*cosPsi - sinPhi*cosTheta*sinPsi;
  Rbg_prime(1, 0) = sinPhi*sinPsi - cosPhi*cosTheta*cosPsi;
  Rbg_prime(1, 1) = -sinPhi*cosPsi - cosPhi*cosTheta*sinPsi;
  Rbg_prime(2, 0) = sinTheta*cosPsi;
  Rbg_prime(2, 1) = sinTheta*sinPsi;*/
	
	// Diebel eq 71
	Rbg_prime(0, 0) = -cosTheta * sinPsi;
	Rbg_prime(0, 1) = cosTheta * cosPsi;
	Rbg_prime(1, 0) = -sinPhi * sinTheta*sinPsi - cosPhi * cosPsi;
	Rbg_prime(1, 1) = sinPhi * sinTheta*cosPsi - cosPhi * sinPsi;
	Rbg_prime(2, 0) = -cosPhi * sinTheta*sinPsi + sinPhi * cosPsi;
	Rbg_prime(2, 1) = cosPhi * sinTheta*cosPsi + sinPhi * sinPsi;
	Rbg_prime = Rbg_prime.T();

  Vector<float, 3> u03dt;
  u03dt(0) = accel[0] * dt;
  u03dt(1) = accel[1] * dt;
  u03dt(2) = accel[2] * dt;

  Vector<float, 3> Rbg_prime_times_u03_dt = Rbg_prime*u03dt;
  
  SquareMatrix<float, QUAD_EKF_NUM_STATES> gPrime;
  gPrime.setIdentity();
  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = Rbg_prime_times_u03_dt(0);
  gPrime(4, 6) = Rbg_prime_times_u03_dt(1);
  gPrime(5, 6) = Rbg_prime_times_u03_dt(2);

  cov = gPrime * cov * gPrime.T() + Q;
	state = newState;
}

void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  Vector<float, 6> z, hOfU;
  z(0) = pos.x;
  z(1) = pos.y;
  z(2) = pos.z;
  z(3) = vel.x;
  z(4) = vel.y;
  z(5) = vel.z;

  for (int i = 0; i < 6; i++)
  {
    hOfU(i) = state(i);
  }

  Matrix<float, 6, 7> hPrime;
	hPrime.setZero();
	for (int i = 0; i < 6; i++)
	{
		hPrime(i,i) = 1;
	}

  Update(z, hPrime, R_GPS, hOfU);
}

void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
	Vector<float, 1> z, hOfU;
	z(0) = magYaw;

	// bring measurement closer to current state to avoid loop-around strangeness
	float diff = z(0) - state(6);
	if (diff < -F_PI) z(0) += 2.f*F_PI;
	if (diff > F_PI) z(0) -= 2.f*F_PI;

	hOfU(0) = state(6);

	Matrix<float, 1, 7> hPrime;
	hPrime.setZero();
	hPrime(0, 6) = 1;

	Update(z, hPrime, R_Yaw, hOfU);
}

template<size_t numZ>
void QuadEstimatorEKF::Update(Vector<float, numZ>& z,
  Matrix<float, numZ, QUAD_EKF_NUM_STATES>& H,
  Matrix<float, numZ, numZ>& R,
  Vector<float, numZ>& hOfU)
{
  SquareMatrix<float, numZ> toInvert;
  toInvert = H*cov*H.T() + R;
  Matrix<float, QUAD_EKF_NUM_STATES,numZ> K = cov * H.T() * inv(toInvert);

  state = state + K*(z - hOfU);

  SquareMatrix<float, QUAD_EKF_NUM_STATES> eye;
  eye.setIdentity();

  cov = (eye - K*H)*cov;
}

float QuadEstimatorEKF::CovConditionNumber() const
{
	MatrixXf m(7, 7);
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			m(i, j) = cov(i, j);
		}
	}

	Eigen::JacobiSVD<MatrixXf> svd(m);
	float cond = svd.singularValues()(0)
		/ svd.singularValues()(svd.singularValues().size() - 1);
	return cond;
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

		GETTER_HELPER("Est.E.x", trueError(0));
		GETTER_HELPER("Est.E.y", trueError(1));
		GETTER_HELPER("Est.E.z", trueError(2));
		GETTER_HELPER("Est.E.vx", trueError(3));
		GETTER_HELPER("Est.E.vy", trueError(4));
		GETTER_HELPER("Est.E.vz", trueError(5));
		GETTER_HELPER("Est.E.yaw", trueError(6));
		GETTER_HELPER("Est.E.pitch", pitchErr);
		GETTER_HELPER("Est.E.roll", rollErr);

		GETTER_HELPER("Est.D.cov_cond", CovConditionNumber());
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

	ret.push_back(_name + ".Est.E.x");
	ret.push_back(_name + ".Est.E.y");
	ret.push_back(_name + ".Est.E.z");
	ret.push_back(_name + ".Est.E.vx");
	ret.push_back(_name + ".Est.E.vy");
	ret.push_back(_name + ".Est.E.vz");
	ret.push_back(_name + ".Est.E.yaw");
	ret.push_back(_name + ".Est.E.pitch");
	ret.push_back(_name + ".Est.E.roll");

	ret.push_back(_name + ".Est.D.cov_cond");

  // diagnostic variables
  ret.push_back(_name + ".Est.D.AccelPitch");
  ret.push_back(_name + ".Est.D.AccelRoll");
	ret.push_back(_name + ".Est.D.ax_g");
	ret.push_back(_name + ".Est.D.ay_g");
	ret.push_back(_name + ".Est.D.az_g");
  return ret;
};
