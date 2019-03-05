#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();
  
  // variables needed for integral control
  integratedAltitudeError = 0;
  
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
  
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
  
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);
  
  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);
  
  kpPQR = config->Get(_config + ".kpPQR", V3F());
  
  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);
  
  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);
  
  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to
  //   individual motor thrust commands
  // INPUTS:
  //   desCollectiveThrust: desired collective thrust [N]
  //   desMoment: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
  
  // HINTS:
  // - you can access parts of desMoment via e.g. desMoment.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
  
  // Collective thrust:                m * c = (F_0 + F_1 + F_2 + F_3)
  // Moment_x (roll,  left vs. right): tau_x = (F_0 - F_1 + F_2 - F_3) * l
  // Moment_y (pitch, front vs. rear): tau_y = (F_0 + F_1 - F_2 - F_3) * l
  // Moment_z (yaw,   cw vs. ccw):     tau_z = (F_0 - F_1 - F_2 + F_3) * -kappa
  
  // [+1, +1, +1, +1] [F_0]   [F_c]
  // [+1, -1, +1, -1] [F_1] = [F_x]
  // [+1, +1, -1, -1] [F_2] = [F_y] => A * F_0123 = F_cxyz
  // [+1, -1, -1, +1] [F_3]   [F_z]
  
  // Calculate inverse of A, A^-1, to determine F_0123 = A^-1 * F_cxyz .
  // In this case: A^-1 = 0.25 * A .
  
  float const moment_arm = L * (float)M_SQRT1_2;    // moment arm: l (used for roll and pitch)
  float const force_c = collThrustCmd;              // force, collective: F_c
  float const force_x = momentCmd.x / moment_arm;   // force, roll:       F_x = tau_x / l
  float const force_y = momentCmd.y / moment_arm;   // force, pitch:      F_y = tau_y / l
  float const force_z = momentCmd.z / -kappa;       // force, yaw:        F_z = tau)z / -kappa
  
  cmd.desiredThrustsN[0] = 0.25f * (force_c + force_x + force_y + force_z); // front left
  cmd.desiredThrustsN[1] = 0.25f * (force_c - force_x + force_y - force_z); // front right
  cmd.desiredThrustsN[2] = 0.25f * (force_c + force_x - force_y - force_z); // rear left
  cmd.desiredThrustsN[3] = 0.25f * (force_c - force_x - force_y + force_z); // rear right
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  // // Convert desired moment into differential thrusts
  // V3F diffThrust;
  
  // // for X shaped quad
  // diffThrust.x = momentCmd.x / L / 2.f / sqrtf(2);
  // diffThrust.y = momentCmd.y / L / 2.f / sqrtf(2);
  // diffThrust.z = momentCmd.z / 4.f / kappa;
  
  // // MIXING
  // // combine the collective thrust with the differential thrust commands to find desired motor thrusts
  // // X Shaped Quad (NED Frame)
  // cmd.desiredThrustsN[0] = collThrustCmd / 4.f - diffThrust.z + diffThrust.y + diffThrust.x; // front left
  // cmd.desiredThrustsN[1] = collThrustCmd / 4.f + diffThrust.z + diffThrust.y - diffThrust.x; // front right
  // cmd.desiredThrustsN[2] = collThrustCmd / 4.f + diffThrust.z - diffThrust.y + diffThrust.x; // rear left
  // cmd.desiredThrustsN[3] = collThrustCmd / 4.f - diffThrust.z - diffThrust.y - diffThrust.x; // rear right
  
  //////////////////////////////// END SOLUTION ///////////////////////////////
  
  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS:
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes
  
  // HINTS:
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)
  
  V3F momentCmd;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  V3F const momentOfInertia = V3F(Ixx, Iyy, Izz);
  
  // Proportional controller
  momentCmd = kpPQR * momentOfInertia * (pqrCmd - pqr);
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  // V3F rate_error = pqrCmd - pqr;
  // V3F omega_dot_des = rate_error * kpPQR;
  // momentCmd = omega_dot_des * V3F(Ixx, Iyy, Izz);
  //////////////////////////////// END SOLUTION ///////////////////////////////
  
  return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS:
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)
  
  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first
  
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // If there is no thrust, there is no way to control roll or pitch.
  if (collThrustCmd <= 0.0f) {
    return V3F();
  }
  
  // Acceleration due to collective thrust
  float const c = collThrustCmd / mass;
  
  // Current direction components: roll(x), pitch(y), yaw(z)
  float const b_x = R(0,2);
  float const b_y = R(1,2);
  float const b_z = R(2,2);
  
  // Commanded/target direction components: roll(x), pitch(y)
  float const b_x_c = -CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
  float const b_y_c = -CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
  
  // Error in direction components: roll(x), pitch(y)
  float const b_x_err = b_x_c - b_x;
  float const b_y_err = b_y_c - b_y;
  
  // Proportional controller terms
  float const b_x_p_term = kpBank * b_x_err;
  float const b_y_p_term = kpBank * b_y_err;
  
  // Computation of angular rate (velocity) commands from P controller terms
  pqrCmd.x = (1/b_z) * (R(1, 0) * b_x_p_term + -R(0, 0) * b_y_p_term);
  pqrCmd.y = (1/b_z) * (R(1, 1) * b_x_p_term + -R(0, 1) * b_y_p_term);
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  
  
  // float target_R13 = -CONSTRAIN(accelCmd[0] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle);
  // float target_R23 = -CONSTRAIN(accelCmd[1] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle);
  
  // if (collThrustCmd < 0)
  // {
  //   target_R13 = 0;
  //   target_R23 = 0;
  // }
  // pqrCmd.x = (1 / R(2, 2))*(-R(1, 0) * kpBank*(R(0, 2) - target_R13) + R(0, 0) * kpBank*(R(1, 2) - target_R23));
  // pqrCmd.y = (1 / R(2, 2))*(-R(1, 1) * kpBank*(R(0, 2) - target_R13) + R(0, 1) * kpBank*(R(1, 2) - target_R23));
  
  //////////////////////////////// END SOLUTION ///////////////////////////////
  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical
  //   acceleration feed-forward command
  // INPUTS:
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]
  
  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER
  
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // Current direction component: yaw(z)
  float const b_z = R(2,2);
  
  float const pos_err = posZCmd - posZ; // position error
  float const vel_err = velZCmd - velZ; // velocity error
  
  integratedAltitudeError += pos_err * dt;  // integration error accumulation
  
  // Proportional–integral–derivative controller terms
  float const p_term = kpPosZ * pos_err;
  float const d_term = kpVelZ * vel_err;
  float const i_term = KiPosZ * integratedAltitudeError;
  
  // Proportional–integral–derivative (PID) controller for vertical acceleration
  float const u_1_bar = p_term + d_term + accelZCmd + i_term;
  
  // Vertical acceleration command (_c), after factoring gravity and yaw.
  float const c_c = ( u_1_bar - CONST_GRAVITY ) / b_z;
  
  // Respective thrust command, after limiting ascent/descent rates.
  thrust = -mass * CONSTRAIN(c_c, -maxAscentRate / dt, maxDescentRate / dt);
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  
  // velZCmd += kpPosZ * (posZCmd - posZ);
  
  // integratedAltitudeError += (posZCmd - posZ) * dt;
  
  // velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  
  // float desAccel = kpVelZ * (velZCmd - velZ) + KiPosZ * integratedAltitudeError + accelZCmd - 9.81f;
  
  // thrust = -(desAccel / R(2, 2) * mass);
  
  //////////////////////////////// END SOLUTION ///////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS:
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations.
  //     the Z component should be 0
  // HINTS:
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY
  
  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;
  
  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  if (velCmd.mag() > maxSpeedXY) {
    velCmd = maxSpeedXY * velCmd.norm();
  }
  
  V3F const pos_err = posCmd - pos; // position error
  V3F const vel_err = velCmd - vel; // velocity error
  
  // Proportional-deriative controller with feed-forward
  accelCmd += (kpPosXY * pos_err) + (kpVelXY * vel_err);
  
  if (accelCmd.mag() > maxAccelXY) {
    accelCmd = maxAccelXY * accelCmd.norm();
  }
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  
  // velCmd += kpPosXY * (posCmd - pos);
  
  // if (velCmd.mag() > maxSpeedXY)
  // {
  //   velCmd = velCmd * maxSpeedXY / velCmd.mag();
  // }
  
  // accelCmd += kpVelXY * (velCmd - vel);
  // if (accelCmd.mag() > maxAccelXY)
  // {
  //   accelCmd = accelCmd * maxAccelXY / accelCmd.mag();
  // }
  
  //////////////////////////////// END SOLUTION ///////////////////////////////
  
  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS:
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS:
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b].
  //  - use the yaw control gain parameter kpYaw
  
  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  float const twoPi = 2.0f * F_PI;
  float yaw_error = fmodf(yawCmd - yaw, twoPi);
  if (fabs(yaw_error) > F_PI) {
    yaw_error += (twoPi) * ((yaw_error > F_PI) ? -1 : 1);
  }
  
  // Proportional controller
  yawRateCmd = kpYaw * yaw_error;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  
  // float yawError = yawCmd - yaw;
  // yawError = fmodf(yawError, F_PI*2.f);
  // if (yawError > F_PI)
  // {
  //   yawError -= 2.f * F_PI;
  // }
  // else if (yawError < -F_PI)
  // {
  //   yawError += 2.f * F_PI;
  // }
  // yawRateCmd = yawError * kpYaw;
  
  //////////////////////////////// END SOLUTION ///////////////////////////////
  
  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);
  
  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);
  
  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());
  
  V3F desMoment = BodyRateControl(desOmega, estOmega);
  
  return GenerateMotorCommands(collThrustCmd, desMoment);
}
