#include <ICM_20948.h>
ICM_20948_I2C IMU;

#define micros2secs 1.0/1000000.0
#define loop_freq 100

#define SERIAL_PORT Serial
#define WIRE_PORT Wire1

#define maxRoll 30.0     //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode 
#define maxPitch 30.0    //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define maxYaw 160.0     //Max yaw rate in deg/sec

//IMU:
#define ACCEL_SCALE_FACTOR 1000.0
#define deg2rad 0.01745329252
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

//Magnetometer:
float MagX, MagY, MagZ;
float MagErrorX = 0.0;
float MagErrorY = 0.0;
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//Radio Variables
#define PPM_Pin 23

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;

//General Global variables
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Orientation
float roll_IMU, pitch_IMU, yaw_IMU;
float q0 = 1.0f; //initialize quaternion
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

void setup() {
  // put your setup code here, to run once:
  SERIAL_PORT.begin(57600); //usb serial

  //Initialize all pins
  pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify
  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(13, HIGH);

  delay(10);
  //Initialize ICM20948 IMU
  imuSetup();

  //Initialize radio communication
  radioSetup();

  //Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
}

void loop() {
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) * micros2secs;

  loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds
  
  //Get vehicle commands
  getCommands(); //pulls current available radio commands
  failSafe(); //prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
  //Compute desired state
  getDesState(); //convert raw commands to normalized values based on saturated control limits

  //Get IMU Data
  getIMUdata();

  //Madgwick Sensor Fusion
  //Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ+1.0, MagY, -MagX, MagZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ + 1.0, dt);

  //printRadioData(100);
  //printDesiredState(100);
  //printGyroData(100);
  //printAccelData(100);
  //printMagData(100);
  //printIMUdata(100);
  //printMadgwickRollPitchYaw(100);
  //printLoopRate(10);

  //Regulate loop rate
  loopWait();
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

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if ( (current_time - blink_counter) * micros2secs > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 0.8;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 1;
      }
  }
}
