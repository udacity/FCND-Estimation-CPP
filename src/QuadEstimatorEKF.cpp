#include "Common.h"
#include "QuadEstimatorEKF.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Math/Quaternion.h"

using namespace SLR;

const int QuadEstimatorEKF::QUAD_EKF_NUM_STATES;

QuadEstimatorEKF::QuadEstimatorEKF(string config, string name) 
  : BaseQuadEstimator(config) ,
	  state(QUAD_EKF_NUM_STATES),
	  cov(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
		Q(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
		R_GPS(6,6),
		R_Yaw(1,1),
		trueError(QUAD_EKF_NUM_STATES)

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
  dtIMU = paramSys->Get(_config + ".dtIMU", .002f);

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

	rollErr = pitchErr = maxEuler = 0;
}

void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
	// Improve a complementary filter-type attitude filter
	// 
	// Currently a small-angle approximation integration method is implemented
	// The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
	// 
	// Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and state(6))
	// to integrate the body rates into new Euler angles.

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	// SMALL ANGLE GYRO INTEGRATION:
	// (replace the code below)
	// make sure you comment it out when you add your own code -- otherwise e.g. you might integrate yaw twice
	
	/*float predictedPitch = pitchEst + dtIMU * gyro.y;
	float predictedRoll = rollEst + dtIMU * gyro.x;
	state(6) = state(6) + dtIMU * gyro.z;	// yaw

	// normalize yaw to -pi .. pi
	if (state(6) > F_PI) state(6) -= 2.f*F_PI;
	if (state(6) < -F_PI) state(6) += 2.f*F_PI; */
	
	/////////////////////////////// END STUDENT CODE ////////////////////////////

	/////////////////////////////// BEGIN SOLUTION //////////////////////////////

	Quaternion<float> quat = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, state(6));
	quat.IntegrateBodyRate(gyro, dtIMU);

	float predictedPitch = quat.Pitch();
	float predictedRoll = quat.Roll();
	state(6) = quat.Yaw();

	//////////////////////////////// END SOLUTION ///////////////////////////////

	// CALCULATE UPDATE
	accelRoll = atan2f(accel.y, accel.z);
	accelPitch = atan2f(-accel.x, 9.81f);

	// FUSE INTEGRATION AND UPDATE
	rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll) + dtIMU / (attitudeTau + dtIMU) * accelRoll;
  pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch) + dtIMU / (attitudeTau + dtIMU) * accelPitch;

	lastGyro = gyro;
}

void QuadEstimatorEKF::UpdateTrueError(V3F truePos, V3F trueVel, Quaternion<float> trueAtt)
{
	VectorXf trueState(QUAD_EKF_NUM_STATES);
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
	maxEuler = MAX(fabs(pitchErr), MAX(fabs(rollErr), fabs(trueError(6))));
}


void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
	VectorXf newState = state;

  // note attitude pitch/roll is already done in "UpdateFromIMU"

	// HINTS
	// if you want to transpose a matrix in-place, use cov.transposeInPlace(), not A = A.transpose()
	// 
	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	/////////////////////////////// BEGIN SOLUTION //////////////////////////////
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

  // TRANSITION MATRIX JACOBIAN

  // first, figure out the Rbg_prime
	MatrixXf Rbg_prime(3,3);
  float sinPhi =		sin(rollEst),		cosPhi = cos(rollEst);		// roll
  float sinTheta =	sin(pitchEst),	cosTheta = cos(pitchEst);	// pitch
  float sinPsi=			sin(state(6)),	cosPsi = cos(state(6));		// yaw
  Rbg_prime.setZero();
	
	// Diebel eq 71.. transposed
	Rbg_prime(0, 0) = -cosTheta * sinPsi;
	Rbg_prime(0, 1) = cosTheta * cosPsi;
	Rbg_prime(1, 0) = -sinPhi * sinTheta*sinPsi - cosPhi * cosPsi;
	Rbg_prime(1, 1) = sinPhi * sinTheta*cosPsi - cosPhi * sinPsi;
	Rbg_prime(2, 0) = -cosPhi * sinTheta*sinPsi + sinPhi * cosPsi;
	Rbg_prime(2, 1) = cosPhi * sinTheta*cosPsi + sinPhi * sinPsi;
	Rbg_prime.transposeInPlace();

	VectorXf u03dt(3);
  u03dt(0) = accel[0] * dt;
  u03dt(1) = accel[1] * dt;
  u03dt(2) = accel[2] * dt;

	VectorXf Rbg_prime_times_u03_dt = Rbg_prime*u03dt;
  
	MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  gPrime.setIdentity();
  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = Rbg_prime_times_u03_dt(0);
  gPrime(4, 6) = Rbg_prime_times_u03_dt(1);
  gPrime(5, 6) = Rbg_prime_times_u03_dt(2);

  cov = gPrime * cov * gPrime.transpose() + Q;
	//////////////////////////////// END SOLUTION ///////////////////////////////

	state = newState;
}

void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  VectorXf z(6), hOfU(6);
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

  MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
	hPrime.setZero();
	for (int i = 0; i < 6; i++)
	{
		hPrime(i,i) = 1;
	}

  Update(z, hPrime, R_GPS, hOfU);
}

void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
	VectorXf z(1), hOfU(1);
	z(0) = magYaw;

	// bring measurement closer to current state to avoid loop-around strangeness
	float diff = z(0) - state(6);
	if (diff < -F_PI) z(0) += 2.f*F_PI;
	if (diff > F_PI) z(0) -= 2.f*F_PI;

	hOfU(0) = state(6);

	MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
	hPrime.setZero();
	hPrime(0, 6) = 1;

	Update(z, hPrime, R_Yaw, hOfU);
}

void QuadEstimatorEKF::Update(VectorXf& z, MatrixXf& H, MatrixXf& R, VectorXf& hOfU)
{
	assert(z.size() == H.rows());
	assert(QUAD_EKF_NUM_STATES == H.cols());
	assert(z.size() == R.rows());
	assert(z.size() == R.cols());
	assert(z.size() == hOfU.size());


/*template<size_t numZ>
void QuadEstimatorEKF::Update(Vector<float, numZ>& z,
  Matrix<float, numZ, QUAD_EKF_NUM_STATES>& H,
  Matrix<float, numZ, numZ>& R,
  Vector<float, numZ>& hOfU)
{*/
  MatrixXf toInvert(z.size(),z.size());
  toInvert = H*cov*H.transpose() + R;
  MatrixXf K = cov * H.transpose() * toInvert.inverse();

  state = state + K*(z - hOfU);

	MatrixXf eye(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
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
		GETTER_HELPER("Est.E.MaxEuler", maxEuler);

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

	ret.push_back(_name + ".Est.E.MaxEuler");

	ret.push_back(_name + ".Est.D.cov_cond");

  // diagnostic variables
  ret.push_back(_name + ".Est.D.AccelPitch");
  ret.push_back(_name + ".Est.D.AccelRoll");
	ret.push_back(_name + ".Est.D.ax_g");
	ret.push_back(_name + ".Est.D.ay_g");
	ret.push_back(_name + ".Est.D.az_g");
  return ret;
};
