#define rad2deg 57.29577951
#define deg2rad 0.01745329252
#define micros2secs 1.0/1000000.0
#define secs2nsecs 1000000000.0
#define ACCEL_SCALE_FACTOR 1000.0 * 9.81
#define deg2rad 0.01745329252
#define N_ESC 6
#define DSHOT_DELAY 1500
#define CRASH_THRES 300

#define ROS_COMM false     //One serial is used for both debugging and ros communication

#define loop_freq 100

#define BAUD_RATE 500000
#define SERIAL_PORT Serial
#define WIRE_PORT Wire1

#define K_p 1.0 // position gain
#define K_v 5.0 // velocity gain
#define K_pI 0.0 // position integral gain
#define K_R 150.0 // orientation gain
#define K_omega 20.0 // angular vel gain
#define K_RI 37 // orientation integral gain
#define yaw_ctrl_scale 1.0 // yaw should be sluggish

#define mass 1.11724 //kg

#define Ixx 0.00388 //kg-m2
//#define Ixx 0.006 //kg-m2 (mr^2 adjusted 1DOF testing without up board)
//#define Ixx 0.01036 //kg-m2 (mr^2 adjusted 1DOF testing with Up board)

#define Iyy 0.00592 //kg-m2
//#define Iyy 0.008 //kg-m2 (mr^2 adjusted 1DOF testing without up board)

#define Izz 0.00779 //kg-m2

#define Ixy 1*0.0000106 //kg-m2
#define Ixz 1*-0.000742 //kg-m2 
#define Iyz 1*-0.0000325 //kg-m2 

#define Lx 0.0635// m (pitch moment arm)
#define Ly 0.085725// m (roll moment arm)
//#define k_f 10 // rotor force coeff
//#define k_m 1 // rotor moment coeff
#define kmf 1.949E-02 // k_m/k_f
#define MTS 155.8 //Gemfan
#define MTC 438.6 //Gemfan
//#define MTS 149.2 //Lum_3_blade
//#define MTC 475.2 //Lum_3_blade
//#define MTS 148.9 //Lum_4_blade
//#define MTC 490.5 //Lum_4_blade

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
#define gyro_noise 0.01 // rad/s
#define VIO_noise 0.002 // m/s

//not used, just for reference (these pins cannot be changed)
#define m1_PIN 4  //
#define m2_PIN 8  //
#define m3_PIN 22 //
#define m4_PIN 9  //

#define PPM_Pin 6

#define maxv 4.0
#define maxRoll 20.0 * deg2rad     //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode 
#define maxPitch 20.0 * deg2rad    //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define maxYaw 160.0 * deg2rad     //Max yaw angle in degrees
#define maxRollRate 360.0 * deg2rad
#define maxPitchRate 360.0 * deg2rad
#define maxYawRate 120.0 * deg2rad

#define unsafeOrient 60.0 * deg2rad
