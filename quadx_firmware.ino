#include <ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <BasicLinearAlgebra.h>
#include <Geometry.h> // Geometry library uses basiclinearalgebra (not matrixmath)
#include "VehicleParameters.h"
#include "DSHOT.h"
#include <ICM_20948.h>

ros::NodeHandle  nh;
ICM_20948_I2C IMU;

uint16_t cmd[N_ESC];
uint8_t tlm[N_ESC];
bool disarmed = true;
ArrayMatrix<4,4,double> MIX_INV;

bool crashed = false;

//IMU:
Point gravity, Acc, Gyro, AccBias, GyroBias;

//Magnetometer:
Point Mag, MagErr, MagScale;

//Radio Variables
unsigned int mode = 1;

//Reference states:
float thro_des;
Point rpy_des, omega_des, v_des, pos_des;
Rotation R_des;

//States
Point rpy, rpy_UKF;
Matrix<4, 1> q;
Matrix<2, 1> StereoAtt;
Rotation R;
Point omega;
Point body_vel, VIO_pos, VIO_vel;

//Control
Matrix<3, 3> Inertia;
float Thrust;
Point Torque;
Matrix<4, 1> Motor_Speed;
Matrix<4, 1> Motor_Signal;
Point CD, rCP, rIMU;

//General Global variables
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter;
float blink_delay;
bool blinkAlternate;

void gv_init() {
  Acc.Fill(0.0);
  Gyro.Fill(0.0);
  AccBias.X() = -0.25;
  AccBias.Y() = 0.15;
  AccBias.Z() = 0.0;
  GyroBias.Fill(0.0);
  Mag.Fill(0.0);
  MagErr.Fill(0.0);
  MagScale.Fill(1.0);
  q(0) = 1.0; //initialize quaternion
  q(1) = 0.0;
  q(2) = 0.0;
  q(3) = 0.0;
  omega.Fill(0.0);
  thro_des = 0.0;
  rpy_des.Fill(0.0);
  v_des.Fill(0.0);
  pos_des.Fill(0.0);
  omega_des.Fill(0.0);
  Thrust = 0.0;
  Torque.Fill(0.0);
  Motor_Speed.Fill(0.0);
  Motor_Signal.Fill(0.0);
  gravity.Fill(0.0);
  gravity(2) = 9.81;
  Inertia = { Ixx, Ixy, Ixz,
              Ixy, Iyy, Iyz,
              Ixz, Iyz, Izz
            };
  CD(0) = CDx;
  CD(1) = CDy;
  CD(2) = CDz;
  rCP(0) = rCPx;
  rCP(1) = rCPy;
  rCP(2) = rCPz;
  rIMU(0) = rIMUx;
  rIMU(1) = rIMUy;
  rIMU(2) = rIMUz;
}

void setup() {

#if (!ROS_COMM)
  SERIAL_PORT.begin(57600); //usb serial
#else
  ROS_CommSetup();
#endif

  //Initialize Global Variables
  gv_init();

  //Initialize all pins
  pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify
  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(13, HIGH);

  delay(10);
  //Initialize ICM20948 IMU
//  imuSetup();

  //Initialize radio communication
  radioSetup();

  //Initialize UKF_Simple
  UKFSetup();

  //Initialize control
  controlSetup();

  //Initialize motors
  motorSetup();
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
}

void loop() {
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) * micros2secs;

  loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds

  getCommands(); //pulls current available radio commands
  failSafe(); //prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
  
  getDesState(); //convert raw commands to normalized values based on saturated control limits

  //Get IMU Data
//  getIMUdata();

  //Madgwick Sensor Fusion
  //Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, MagY, MagX, MagZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
  Madgwick6DOF(Gyro.X(), Gyro.Y(), Gyro.Z(), Acc.X(), Acc.Y(), Acc.Z(), dt);
  R.FromQuaternions(q(0), q(1), q(2), q(3));
  rpy = EulerAnglesFrom(R);

  UKFPropagate();
  UKFUpdate();
  R.FromStereo(StereoAtt, rpy(2));
  rpy_UKF = EulerAnglesFrom(R);

  //Control
  switch (mode) {
    case 0: //ACRO
      controlOmega();
      break;
    case 1: //Manual
      controlSO3();
      break;
    case 2: //Position hold
      controlSE3();
      break;
  }

  Mixer();
  send_motor_cmds();

#if (!ROS_COMM)
  // These are all for debugging purposes only
//      printRadioData(100);
//    printDesiredState(100);
  //  printGyroData(100);
  //    printAccData(100);
  //  printMagData(100);
  //  printAccMagnitude(100);
  //printIMUdata(100);
  //printVIOVelocity(100);
  //      printMadgwickRollPitchYaw(100);
  //  printVisualizationYawPitchRoll(100);
  //  printMadgwickQuaternions(100);
  //  printUKFStereo(100);
  //    printUKFRollPitchYaw(100);
  //printUKFVelocity(100);
  //  printUKFOmega(100);
  //    printTrace(100);
  //      printUKFAccEst(100);
  //    printUKFGyroEst(100);
//  printControlOutput(100);
//  printMotorOutput(100);
  printMotorCmdOutput(100);
//      printLoopRate(10);
  SERIAL_PORT.println();
#else
  // For publishing to a ros topic
  //ros_publish_imu();
  ros_publish_Madg_odom();
  ros_publish_UKF_odom();
  nh.spinOnce();
#endif

  //Regulate loop rate
  loopWait();
}

void printVIOVelocity(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
#define plot_scaling 10
    print_counter = micros();
    SERIAL_PORT.print(F(" VIO_vx : "));
    SERIAL_PORT.print(VIO_vel(0)*plot_scaling);
    SERIAL_PORT.print(F(" VIO_vy: "));
    SERIAL_PORT.print(VIO_vel(1)*plot_scaling);
    SERIAL_PORT.print(F(" VIO_vz: "));
    SERIAL_PORT.print(VIO_vel(2)*plot_scaling);
  }
}

void loopWait() {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
#define invFreq (1.0 / loop_freq)
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)*micros2secs) {
    checker = micros();
  }
}

void printLoopRate(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F("Loop Rate = "));
    SERIAL_PORT.println(1 / dt);
  }
}

void setupBlink(int numBlinks, int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
     It looks cool.
  */
  if (crashed) {
    digitalWrite(13, 1); //pin 13 is built in LED
  }
  else if ( (current_time - blink_counter) * micros2secs > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //pin 13 is built in LED

    if (disarmed) {
      if (blinkAlternate == 1) {
        blinkAlternate = 0;
        blink_delay = 0.3;
      }
      else if (blinkAlternate == 0) {
        blinkAlternate = 1;
        blink_delay = 0.5;
      }
    }
    else {
      if (blinkAlternate == 1) {
        blinkAlternate = 0;
        blink_delay = 0.1;
      }
      else if (blinkAlternate == 0) {
        blinkAlternate = 1;
        blink_delay = 0.2;
      }
    }
  }
}
