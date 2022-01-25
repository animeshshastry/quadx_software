#include <ros.h>
#include <Vector.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <BasicLinearAlgebra.h>
#include <Geometry.h> // Geometry library uses basiclinearalgebra (not matrixmath)
#include "VehicleParameters.h"
#include "DSHOT.h"
#include <ICM_20948.h>

//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 4096> nh;

ICM_20948_I2C IMU;

float avg_m1_volt, avg_m2_volt, avg_m3_volt, avg_m4_volt;
uint16_t m1_dac_setting;
boolean avg_m1_volt_bool[8];

uint16_t cmd[N_ESC];
uint8_t tlm[N_ESC];
bool disarmed = true;
ArrayMatrix<4,4,double> MIX_INV;

bool crashed = false;
bool reset_origin_flag = false;
bool reset_yaw_flag = false;

//IMU:
Point gravity, Acc, Gyro, AccBias, GyroBias;

//Magnetometer:
Point Mag, MagErr, MagScale;

//Radio Variables
unsigned int mode = 1;
double rc_knob = 1.0;

// Freq Response Variables
double w = 0.0, wx = 0.0, wy = 0.0, wz = 0.0, wpsi = 0.0;
double wt = 0.0, wxt = 0.0, wyt = 0.0, wzt = 0.0, wpsit = 0.0;
double t0 = 0.0;
int w_dot_sign = 1;

//Reference states:
float thro_des, yaw_origin;
Point rpy_des, omega_des, acc_des, v_des, pos_des, origin;
Rotation R_des;

//States
Point rpy, rpy_UKF;
Matrix<4, 1> q;
Matrix<2, 1> StereoAtt;
Rotation R;
Point omega;
Point body_vel, wind_vel, VIO_pos, VIO_vel, VIO_Euler;
double trace;
ArrayMatrix<3, 1, double> e3 = {0, 0, 1.0};
int deg, volt, amp, mah, rpm;
float volt_lp = 16.0;

//Control
Matrix<3, 3> Inertia;
float Thrust, Thrust_LP;
Point Torque, Torque_LP;
ArrayMatrix<4, 1, double> Motor_Thrust;
ArrayMatrix<4, 1, double> Motor_RPM_Des_dot;
ArrayMatrix<4, 1, double> Motor_RPM_Des;
ArrayMatrix<4, 1, double> Motor_RPM;
Point CD, rCP, rIMU, rVIO;

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
  AccBias.X() = -0.15;
  AccBias.Y() = 0.02;
  AccBias.Z() = -0.04;
  GyroBias.X() = 0.56*deg2rad;
  GyroBias.Y() = 0.60*deg2rad;
  GyroBias.Z() = 0.52*deg2rad;
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
  acc_des.Fill(0.0);
  v_des.Fill(0.0);
  pos_des.Fill(0.0);
  origin.Fill(0.0);
  omega_des.Fill(0.0);
  Thrust = 0.0;
  Thrust_LP = 0.0;
  yaw_origin = 0.0;
  Torque.Fill(0.0);
  Torque_LP.Fill(0.0);
  Motor_Thrust.Fill(0.0);
  Motor_RPM_Des_dot.Fill(0.0);
  Motor_RPM_Des.Fill(0.0);
  Motor_RPM.Fill(0.0);
  gravity.Fill(0.0);
  gravity(2) = 9.80665;
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
  rVIO(0) = rVIOx;
  rVIO(1) = rVIOy;
  rVIO(2) = rVIOz;
}

void setup() {

#if (!ROS_COMM)
  //SERIAL_PORT.begin(57600); //usb serial (baud_rate has no meaning)
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
  imuSetup();

  //Initialize radio communication
  radioSetup();

  //Initialize UKF_Simple
#if (!sq_UKF)
  UKFSetup();
#else
  ukf_init();
#endif

  //Initialize control
  controlSetup();

  //Initialize motors
  motorSetup();

  ESC_TLM_PORT.begin(115200);
  
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
  getIMUdata();
  
  //Get Serial Data
  //read_Serial();

  //Madgwick Sensor Fusion
  //Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, MagY, MagX, MagZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
  Madgwick6DOF(Gyro.X(), Gyro.Y(), Gyro.Z(), Acc.X(), Acc.Y(), Acc.Z(), dt);
//  R.FromQuaternions(q(0), q(1), q(2), q(3));
  rpy = EulerAnglesFrom(R);
  omega = Gyro;

#if (!sq_UKF)
  UKFPropagate();
  UKFUpdate();
#else
  ukf_iterate(dt);
#endif
  
  if (!disarmed){
    //Control
    switch (mode) {
      case 0: //Manual
        R.FromStereo(StereoAtt, 0);
        rpy_UKF = EulerAnglesFrom(R);
        controlSO3();
        break;
      case 1: //Position hold
        R.FromStereo(StereoAtt, VIO_Euler(2));
        rpy_UKF = EulerAnglesFrom(R);
        controlSE3_pos(0.0);
        break;
      case 5: //Position hold with Aero
        R.FromStereo(StereoAtt, VIO_Euler(2));
        rpy_UKF = EulerAnglesFrom(R);
        controlSE3_pos(1.0);
        break;
      case 3: //ACRO
        R.FromStereo(StereoAtt, 0);
        rpy_UKF = EulerAnglesFrom(R);
        controlOmega();
        break;
      case 4: //Velocity hold
        R.FromStereo(StereoAtt, 0);
        rpy_UKF = EulerAnglesFrom(R);
        controlSE3_vel();
        break;
#if (FREQ_RES)
      case 2: //Frequency Response
        float w_dot = 0.001 + 0.1*w;
        if (w > maxw) {
          w_dot_sign = 1;
          w = 0.0;
          t0 = current_time*micros2secs;
        }
        if (w < 0) {
          w_dot_sign = 1;
          t0 = current_time*micros2secs;
        }
        w += w_dot_sign * w_dot * dt;
        wt = w*(current_time*micros2secs - t0);
        wx = w; wy = w; wz = w; wpsi = w;
        wxt = wt; wyt = wt; wzt = wt; wpsit = wt;
        R.FromStereo(StereoAtt, VIO_Euler(2));
        rpy_UKF = EulerAnglesFrom(R);
        controlSE3_pos(0.0);
        break;
#else
      case 2: //Trajectory Tracking
//        wx = 2.0; wy = 4.0; wz = 1.0; wpsi = 0.0;// Figure8
        wx = 2.0; wy = 2.0; wz = 2.0; wpsi = 0.0;// Circle 
        wxt = wx*(current_time*micros2secs - t0);
        wyt = wy*(current_time*micros2secs - t0);
        wzt = wz*(current_time*micros2secs - t0);
        wpsit = wpsi*(current_time*micros2secs - t0);
        R.FromStereo(StereoAtt, VIO_Euler(2));
        rpy_UKF = EulerAnglesFrom(R);
//        controlSE3_pos(0.0);
        controlSE3_pos(1.0);
        break;
#endif
    }
  }

  Thrust_LP = T_LP * Thrust + (1-T_LP)*Thrust_LP;
  Torque_LP = Torque * Tau_LP + Torque_LP * (1-Tau_LP);
  
//  if ( !disarmed && (rpy_UKF(0) > unsafeOrient || rpy_UKF(0) < -unsafeOrient || rpy_UKF(1) > unsafeOrient || rpy_UKF(1) < -unsafeOrient)) {
//    crashed = true;
//  }

  Mixer();
  send_motor_cmds();
//  update_dac_setting();
  
#if (!ROS_COMM)
  // These are all for debugging purposes only
//   printRadioData(100);
    //printDesiredState(100);
//    printGyroData(100);
//     printAccData(100);
//    printMagData(100);
  //  printAccMagnitude(100);
//  printIMUdata(100);
  //printVIOVelocity(200);
//        printMadgwickRollPitchYaw(100);
//    printVisualizationYawPitchRoll(100);
  //  printMadgwickQuaternions(100);
  //  printUKFStereo(100);
//      printUKFRollPitchYaw(100);
  //printUKFVelocity(100);
//    printUKFOmega(100);
//      printTrace(100);
//        printUKFAccEst(100);
//      printUKFGyroEst(100);
//  printControlOutput(100);
//  printMotorOutput();
//  printMotorCmdOutput(100);
//  printMotorCmdRateOutput();
//   printLoopRate();
//  SERIAL_PORT.println();
#else
  // For publishing to a ros topic
  update_header();
//  ros_publish_imu();
//  ros_publish_Madg_odom();
//  ros_publish_UKF_state_odom();
  ros_publish_UKF_param();
//  ros_publish_Des_odom();
  ros_publish_debug();
  nh.spinOnce();
#endif

//  send_IMU_data();
//  send_Madg_data();
//  send_UKF_state_data();
//  send_UKF_param_data();
//  send_Des_data();
//  SERIAL_PORT.send_now();

  //Regulate loop rate
  loopWait();
}

void loopWait() {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  unsigned long checker = micros();
  //Sit in loop until appropriate time has passed
  while ((checker - current_time)*micros2secs*loop_freq < 1) {  
    checker = micros();
  }
}

void update_dac_setting(){
float volt_lp = 1e-1;
  for (int i = 0; i < 10; i++){
    float val = 3.3*analogRead(21)/1023.0;
    avg_m1_volt = volt_lp*val+(1-volt_lp)*avg_m1_volt;
    m1_dac_setting = avg_m1_volt*(64/3.3)-1;
    convertDecToBin(m1_dac_setting,avg_m1_volt_bool);
    avg_m1_volt_bool[0] = 1;
    avg_m1_volt_bool[1] = 1;
    CMP1_DACCR = convertBinToDec(avg_m1_volt_bool);
  }
}

void printLoopRate() {
//  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
//    print_counter = micros();
    SERIAL_PORT.print(F("Loop Rate = "));
    SERIAL_PORT.println(1.0 / dt);
//  }
}

void printVIOVelocity(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
#define plot_scaling 1
    print_counter = micros();
    SERIAL_PORT.print(F(" VIO_vx : "));
    SERIAL_PORT.print(VIO_vel(0)*plot_scaling);
    SERIAL_PORT.print(F(" VIO_vy: "));
    SERIAL_PORT.print(VIO_vel(1)*plot_scaling);
    SERIAL_PORT.print(F(" VIO_vz: "));
    SERIAL_PORT.print(VIO_vel(2)*plot_scaling);
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
