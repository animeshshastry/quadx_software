#define rad2deg 57.29577951
#define deg2rad 0.01745329252
#define micros2secs 1.0/1000000.0
#define secs2nsecs 1000000000.0
#define ACCEL_SCALE_FACTOR 1000.0 * 9.81
#define deg2rad 0.01745329252
#define N_ESC 6
#define DSHOT_DELAY 1500
#define CRASH_THRES 900

#define ROS_COMM false     //One serial is used for both debugging and ros communication

#define loop_freq 100

#define BAUD_RATE 500000
#define SERIAL_PORT Serial
#define WIRE_PORT Wire1

#define K_p 1.0 // position gain
#define K_v 5.0 // velocity gain
#define K_R 100.0 // orientation gain
#define K_omega 20.0 // angular vel gain

#define mass 1.0 //kg
#define Ixx 0.001 //kg-m2
#define Iyy 0.001 //kg-m2
#define Izz 0.001 //kg-m2
#define Ixy 0.0 //kg-m2
#define Ixz 0.0 //kg-m2 
#define Iyz 0.0 //kg-m2 
#define Lx 0.0635// m (pitch moment arm)
#define Ly 0.085725// m (roll moment arm)
#define k_f 10 // Thrust Slope dT/d(0-1) Units: N/(0-1)
#define k_m 1 // Moment Slope dM/d(0-1) Units: N-m/(0-1)

//Drag coefficient
#define CDx .00
#define CDy .00
#define CDz .00

// Location of Centre of Pressure wrt cg
#define rCPx 0.0 // m
#define rCPy 0.0 // m
#define rCPz 0.0 // m

// Location of IMU wrt cg
#define rIMUx -0.0508 // m
#define rIMUy 0.0 // m
#define rIMUz 0.0 // m

// Sensor noise params
#define acc_noise 0.5 // m/s^2
#define gyro_noise 0.017 // rad/s
#define VIO_noise 0.002 // m/s

//not used, just for reference (these pins cannot be changed)
#define m1_PIN 4  //
#define m2_PIN 8  //
#define m3_PIN 22 //
#define m4_PIN 9  //

#define PPM_Pin 6

#define maxv 4.0
#define maxRoll 30.0 * deg2rad     //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode 
#define maxPitch 30.0 * deg2rad    //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define maxYaw 160.0 * deg2rad     //Max yaw angle in degrees
#define maxOmega 90.0 * deg2rad
