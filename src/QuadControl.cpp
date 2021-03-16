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
    // declaring variables
    float s = collThrustCmd;
    float x = momentCmd.x;
    float y = momentCmd.y;
    float z = momentCmd.z;
    float length=L/pow(2.f, 0.5);
    
    // INPUTS:
    //   collThrustCmd: desired collective thrust [N]
      S = collThrustCmd/4.f;
      
    //   momentCmd: desired rotation moment about each axis [N m]
      X = momentCmd.x/(l*4.f);
      Y = momentCmd.y/(l*4.f);
      Z = momentCmd.z/(kappa*4.f);
      
      F = S + X + Y - Z;
      F1 = S - X + Y + Z;
      F2 = S + X - Y + Z;
      F3 = S - X - Y - Z;
    
    // front left
    cmd.desiredThrustsN[0] = F;
    // front right
    cmd.desiredThrustsN[1] = F2;
    // rear left
    cmd.desiredThrustsN[2] = F3;
    // rear right
    cmd.desiredThrustsN[3] = F4;
    

  /////////////////////////////// END STUDENT CODE ////////////////////////////

    
    
    
    

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate


  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
      float pdot_cmd, qdot_cmd, rdot_cmd;
      float tau_cmd[3];
    //   pqrCmd: desired body rates [rad/s]
      pdot_cmd = kpPQR.x * (pqrCmd.x - pqr.x);
      qdot_cmd = kpPQR.y * (pqrCmd.y - pqr.y);
      rdot_cmd = kpPQR.z * (pqrCmd.z - pqr.z);
      
    //   pqr: current or estimated body rates [rad/s]
      tau_cmd[0] = Ixx * pdot_cmd;
      tau_cmd[1] = Iyy * qdot_cmd;
      tau_cmd[2] = Izz * rdot_cmd;
      
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes
      momentCmd = tau_cmd;
      
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////


  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command


  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)
    
    float c = - collThrustCmd / mass;
    float bx, by;
    float bx_dot, by_dot;
    
    bx = CONSTRAIN((accelCmd.x / c), -sin(maxTiltAngle), sin(maxTiltAngle));
    by = CONSTRAIN((accelCmd.y / c), -sin(maxTiltAngle), sin(maxTiltAngle));
    bx_dot = kpBank * (bx - R (0,2));
    by_dot = kpBank * (by - R (1,2));

    pqrCmd.x = (R(1,0) * bx_dot - R(0,0) * by_dot) / R(2,2);
    pqrCmd.y = (R(1,1) * bx_dot - R(0,1) * by_dot) / R(2,2);
    
    
  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command


  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]


      ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    float pos_err, vel_err, vel_cmd, acc_cmd;

       pos_err = posZCmd - posZ;
       
       integratedAltitudeError += pos_err * dt;
       
       vel_cmd = kpPosZ * pos_err + velZCmd;

       vel_cmd = CONSTRAIN(vel_cmd, -maxAscentRate, maxDescentRate);
       
       vel_err = vel_cmd - velZ;

       acc_cmd = kpVelZ * vel_err + accelZCmd + KiPosZ * integratedAltitudeError;
       
       thrust = mass * (CONST_GRAVITY - acc_cmd)/ R(2,2);
       
       thrust = CONSTRAIN(thrust, 4*minMotorThrust, 4*maxMotorThrust);
       


  /////////////////////////////// END STUDENT CODE ////////////////////////////


  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose

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

    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmdFF: feed-forward acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    
    V3F vel_cmd;
    
    vel_cmd = kpPosXY * (posCmd - pos) + velCmd;
    if (vel_cmd.mag() > maxSpeedXY){
        
        vel_cmd = vel_cmd.norm() * maxSpeedXY;
    }
    
    accelCmd += kpVelXY * (vel_cmd - vel);
    accelCmd.z = 0;
    if (accelCmd.mag() > maxAccelXY ){
        accelCmd = accelCmd.norm() * maxAccelXY;
    }

    

  /////////////////////////////// END STUDENT CODE ////////////////////////////




  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd

  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // INPUTS:
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    
    float err = 0;
    
    if (yawCmd > 0){
        yawCmd = fmodf(yawCmd, 2*F_PI);
    }
    else if (yawCmd < 0){
        yawCmd = -fmodf(-yawCmd, 2*F_PI);
    }
    err = (yawCmd - yaw);
    
    if (err > F_PI){
        err -= 2*F_PI;
        
    }
    else if (err < -F_PI){
        err += 2*F_PI;
    }
    
    yawRateCmd = kpYaw * err;
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  
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
