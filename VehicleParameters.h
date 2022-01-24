// PLEASE DON'T CHANGE THE UNIV CONSTANTS
#define rad2deg 57.29577951
#define deg2rad 0.01745329252
#define micros2secs 1.0/1000000.0
#define secs2nsecs 1000000000.0
#define PI 3.14159265359
#define ACCEL_SCALE_FACTOR 1000.0 * 9.80665
#define deg2rad 0.01745329252
#define N_ESC 6
#define DSHOT_DELAY 150
#define DSHOT_STOP_CMD 0
#define ARMING_REP 1000
#define MOTOR_MAGNETS 14  //Number of magnets in the motor (for converting eRPM to RPM)
#define CRASH_THRES 300   //Crash detection based on error b/w estimation and measurement

#define ROS_COMM true     //One serial is used for both debugging and ros communication
#define FREQ_RES false     // Either Freq Response or Trajectory Tracking
#define sq_UKF true       // Do not compile sq_UKF (Eigen) on an embedded computer
#define wind_est true
#define use_linear_drag false
#define wind_in_body_frame false
#define RPM_TLM true
#define VOLT_CORR true
#define CLOSE_RPM_LOOP false

#define loop_freq 120

#define BAUD_RATE 400000000
#define SERIAL_PORT Serial
#define WIRE_PORT Wire1
#define ESC_TLM_PORT Serial3

#define T_LP 1.0 // Control Low_Pass
#define Tau_LP 1.0 // Control Low_Pass

#define K_p 4.0 // position gain
#define K_v 2.0 // velocity gain
#define K_pI 0.0 // position integral gain
#define K_vI 0.0 // velocity integral gain

#define K_R 100.0 // orientation gain
#define K_omega 15.0 // angular vel gain
#define K_RI 40.0 // orientation integral gain
#define yaw_ctrl_scale 0.25 // to make yaw sluggish
#define K_omegaI 0 // yaw rate integral gain

#define K_RPM 80.0 // RPM proportional control gain (if CLOSE_RPM_LOOP)
#define K_RPM_I 40.0 // RPM integral control gain (if CLOSE_RPM_LOOP)

#define mass 0.9 //kg

#define Ixx 0.00388 //kg-m2
#define Iyy 0.00592 //kg-m2
#define Izz 0.00779 //kg-m2

#define Ixy 1*0.0000106 //kg-m2
#define Ixz 1*-0.000742 //kg-m2 
#define Iyz 1*-0.0000325 //kg-m2 

#define Lx 0.066// m (pitch moment arm)
#define Ly 0.087// m (roll moment arm)

// Old Calibration constants
//#define k_f 10 // rotor force coeff
//#define k_m 1 // rotor moment coeff
//#define kmf 1.949E-02 // k_m/k_f
#define MTS 155.8 //Gemfan
#define MTC 438.6 //Gemfan
//#define MTS 149.2 //Lum_3_blade
//#define MTC 475.2 //Lum_3_blade
//#define MTS 148.9 //Lum_4_blade
//#define MTC 490.5 //Lum_4_blade

// New Calibration Constants for the Gemfan 5043 @ 15V with Lumenier 2450kv
#define k_f 1.717E-08
#define MT0 111.0
#define MT1 0.03953
#define MT2 1.567E-06
#define MT0_2 12.19
#define MT1_2 0.001345
#define kmf 0.010745

#define VOLT_LP 0.01    // Batt Voltage Low Pass
#define RPM_LP 1.0     // Motor RPM Low Pass

//Drag coefficient
# if (!use_linear_drag)
//#define CDx 1*.043
//#define CDy 1*.043
//#define CDz 1*.1
//#define CDx 1*.04    // with Drag Plate
//#define CDy 1*.15     // with Drag Plate
//#define CDz 1*.1    // with Drag Plate
#define CDx .03    // with Box
#define CDy .12     // with Box
#define CDz .2    // with Box
#else
//#define CDx 1*.08
//#define CDy 1*.08
//#define CDz 1*.25
#define CDx .1    // with Box
#define CDy .42     // with Box
#define CDz .6    // with Box
#endif

// Location of Centre of Pressure wrt cg
#define rCPx 0.0 // m
#define rCPy 0.0 // m
//#define rCPz 0.0 // m
#define rCPz -0.08 // m   // with Drag Plate

// Location of IMU wrt cg
#define rIMUx -0.05 // m
#define rIMUy 0.0 // m
#define rIMUz -0.01 // m

// Location of VIO wrt cg
#define rVIOx 0.085 // m
#define rVIOy 0.0 // m
#define rVIOz -0.04 // m

// Sensor noise params
#define acc_noise 0.3 // m/s^2
#define gyro_noise 0.3*deg2rad // rad/s
#define VIO_noise 0.005 // m/s

//not used, just for reference (these pins cannot be changed)
#define m1_PIN 4  //
#define m2_PIN 8  //
#define m3_PIN 22 //
#define m4_PIN 9  //

#define PPM_Pin 6

#define maxpos_x 2.0
#define maxpos_y 2.0
#define maxpos_z 2.0
#define maxv 4.0
#define maxvz 5.0
#define maxRoll 20.0 * deg2rad     //Max roll angle in degrees for orientation control mode (maximum 60 degrees), deg/sec for rate mode 
#define maxPitch 20.0 * deg2rad    //Max pitch angle in degrees for orientation control mode (maximum 60 degrees), deg/sec for rate mode
#define maxYaw 180.0 * deg2rad     //Max yaw angle in degrees
#define maxRollRate 360.0 * deg2rad
#define maxPitchRate 360.0 * deg2rad
#define maxYawRate 120.0 * deg2rad

#define unsafeOrient 90.0 * deg2rad

#define MAX_RPM 20000.0 // Max motor RPM

#define maxw 60.0          // Frequency Response's maximum frequency
