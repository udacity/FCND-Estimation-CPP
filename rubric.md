# Tasks for Estimation exercise

## Task 1: Sensor Noise

For simplicity and shorter code version I decided to follow what was proposed as the second hint using Quaternion:
```
Quaternion<float> q = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
q.IntegrateBodyRate(gyro, dtIMU);
float predictedPitch = q.Pitch();
float predictedRoll = q.Yaw();
```

## Task 2: Prediction Step
This part is traightforward as we only need to update values based on the step time.
```
  predictedState(0) = curState(0) + dt * curState(3);
  predictedState(1) = curState(1) + dt * curState(4);
  predictedState(2) = curState(2) + dt * curState(5);

  V3F accelI = attitude.Rotate_BtoI(accel);

  predictedState(3) = curState(3) + dt * accelI.x;
  predictedState(4) = curState(4) + dt * accelI.y;
  predictedState(5) = curState(5) + dt * accelI.z - dt * CONST_GRAVITY;
```

For Rgb_ the code is also very intuitive and it's just a matter of implmented what was in the lecture  notes:
```
  RbgPrime(0, 0) = -cos(pitch) * sin(yaw);
  RbgPrime(0, 1) = -sin(roll) * sin(pitch) * sin(yaw) - cos(pitch) * cos(yaw);
  RbgPrime(0, 2) = -cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(yaw);
  RbgPrime(1, 0) = cos(pitch) * cos(yaw);
  RbgPrime(1, 1) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
  RbgPrime(1, 2) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
```

For Predict function I just followed what is a page 9:
```
    gPrime(0,3) = dt;
    gPrime(1,4) = dt;
    gPrime(2,5) = dt;
    
    gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;
    gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;
    gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;
    
    ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

## Task 3: Magnetometer Update
Function implemented:
```
  hPrime(6) = 1;

  zFromX(0) = ekfState(6);
  float diff = magYaw - ekfState(6);
  if (diff > F_PI) {
      zFromX(0) += 2.f * F_PI;
  }
  else if (diff < -F_PI) {
      zFromX(0) -= 2.f * F_PI;
  }
```

## Task 4: Closed Loop + GPS Update
Implementation:
```
  hPrime.topLeftCorner(QUAD_EKF_NUM_STATES - 1, QUAD_EKF_NUM_STATES - 1) = MatrixXf::Identity(QUAD_EKF_NUM_STATES - 1, QUAD_EKF_NUM_STATES - 1);
  zFromX = hPrime * ekfState;
```