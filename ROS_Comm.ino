//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry

//========================================================================================================================//

#if (ROS_COMM)

std_msgs::Header header;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
std_msgs::Float64MultiArray array_msg;

ros::Publisher imu_pub("imu_raw", &imu_msg);
ros::Publisher Madg_odom_pub("odom_Madg", &odom_msg);
ros::Publisher UKF_state_odom_pub("odom_UKF_state", &odom_msg);
ros::Publisher UKF_param_odom_pub("odom_UKF_param", &odom_msg);
ros::Publisher Des_odom_pub("odom_Des", &odom_msg);
ros::Publisher debug_pub("debug", &array_msg);
//ros::Subscriber<nav_msgs::Odometry> odom_sub("/camera/odom/sample", vio_callback );
//ros::Subscriber<nav_msgs::Odometry> odom_sub("/VIO_odom", vio_callback );
ros::Subscriber<geometry_msgs::Point> pos_sub("/VIO_pos", pos_callback );
ros::Subscriber<geometry_msgs::Vector3> vel_sub("/VIO_vel", vel_callback );
ros::Subscriber<geometry_msgs::Vector3> Euler_sub("/VIO_Euler", Euler_callback );

ros::Subscriber<geometry_msgs::Twist> Pos_and_vel_sub("/rs_t265/position_and_velocity", Pos_and_vel_callback );
ros::Subscriber<geometry_msgs::Vector3> Attitude_sub("/rs_t265/attitude", Attitude_callback );

ros::Subscriber<geometry_msgs::Pose> Pose_sub("/rs_t265/pose", Pose_callback );

void ROS_CommSetup() {
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
//  nh.advertise(imu_pub);
//  nh.advertise(Madg_odom_pub);
//  nh.advertise(UKF_state_odom_pub);
  nh.advertise(UKF_param_odom_pub);
//  nh.advertise(Des_odom_pub);
  nh.advertise(debug_pub);
  
  //nh.subscribe(odom_sub);
  
//  nh.subscribe(pos_sub);
//  nh.subscribe(vel_sub);
//  nh.subscribe(Euler_sub);

//  nh.subscribe(Pos_and_vel_sub);
//  nh.subscribe(Attitude_sub);

  nh.subscribe(Pose_sub);
}

void update_header(){
  header.frame_id = "camera_odom_frame";
  header.stamp = nh.now();

  imu_msg.header = header;
  odom_msg.header = header; 
}

void Pose_callback( const geometry_msgs::Pose& msg) {
  if (reset_origin_flag) {
    origin.X() = msg.position.x;
    origin.Y() = msg.position.y;
    origin.Z() = msg.position.z;
    reset_origin_flag = false;
  }
  VIO_pos(0) = (msg.position.x - origin.X())*cos(yaw_origin) + (msg.position.y - origin.Y())*sin(yaw_origin);
  VIO_pos(1) = -(msg.position.x - origin.X())*sin(yaw_origin) + (msg.position.y - origin.Y())*cos(yaw_origin);
  VIO_pos(2) = msg.position.z - origin.Z();
  
  VIO_vel(0) = msg.orientation.x;
  VIO_vel(1) = msg.orientation.y;
  VIO_vel(2) = msg.orientation.z;

  VIO_vel = VIO_vel + omega.CrossProduct(rVIO);

  if (reset_yaw_flag) {
    yaw_origin = msg.orientation.w;
    reset_yaw_flag = false;
  }
  VIO_Euler(2) = msg.orientation.w - yaw_origin;
  
}

void Pos_and_vel_callback( const geometry_msgs::Twist& msg) {
  if (reset_origin_flag) {
    origin.X() = msg.linear.x;
    origin.Y() = msg.linear.y;
    origin.Z() = msg.linear.z;
    reset_origin_flag = false;
  }
  VIO_pos(0) = (msg.linear.x - origin.X())*cos(yaw_origin) + (msg.linear.y - origin.Y())*sin(yaw_origin);
  VIO_pos(1) = -(msg.linear.x - origin.X())*sin(yaw_origin) + (msg.linear.y - origin.Y())*cos(yaw_origin);
  VIO_pos(2) = msg.linear.z - origin.Z();

//  VIO_pos(0) = msg.linear.x;
//  VIO_pos(1) = msg.linear.y;
//  VIO_pos(2) = msg.linear.z;
  
  VIO_vel(0) = msg.angular.x;
  VIO_vel(1) = msg.angular.y;
  VIO_vel(2) = msg.angular.z;
}

void Attitude_callback( const geometry_msgs::Vector3& msg) {
  if (reset_yaw_flag) {
    yaw_origin = msg.z;
    reset_yaw_flag = false;
  }
  VIO_Euler(0) = msg.x;
  VIO_Euler(1) = msg.y;
  VIO_Euler(2) = msg.z - yaw_origin;
}

void vio_callback( const nav_msgs::Odometry& msg) {
  //odom_msg.header = msg.header;
  VIO_pos(0) = msg.pose.pose.position.x;
  VIO_pos(1) = msg.pose.pose.position.y;
  VIO_pos(2) = msg.pose.pose.position.z;
  VIO_vel(0) = msg.twist.twist.linear.x;
  VIO_vel(1) = msg.twist.twist.linear.y;
  VIO_vel(2) = msg.twist.twist.linear.z;
  //Serial.println(msg.twist.twist.linear.x);
}

void pos_callback( const geometry_msgs::Point& msg) {
  if (reset_origin_flag) {
    origin.X() = msg.x;
    origin.Y() = msg.y;
    origin.Z() = msg.z;
    reset_origin_flag = false;
  }
  VIO_pos(0) = (msg.x - origin.X())*cos(yaw_origin) + (msg.y - origin.Y())*sin(yaw_origin);
  VIO_pos(1) = -(msg.x - origin.X())*sin(yaw_origin) + (msg.y - origin.Y())*cos(yaw_origin);
  VIO_pos(2) = msg.z - origin.Z();
}

void vel_callback( const geometry_msgs::Vector3& msg) {
  VIO_vel(0) = msg.x;
  VIO_vel(1) = msg.y;
  VIO_vel(2) = msg.z;
}

void Euler_callback( const geometry_msgs::Vector3& msg) {
  if (reset_yaw_flag) {
    yaw_origin = msg.z;
    reset_yaw_flag = false;
  }
  VIO_Euler(0) = msg.x;
  VIO_Euler(1) = msg.y;
  VIO_Euler(2) = msg.z - yaw_origin;
}

void ros_publish_Des_odom() {
  
  Point rpy_des_temp = EulerAnglesFrom(R_des);
  
  odom_msg.pose.pose.position.x = pos_des(0);
  odom_msg.pose.pose.position.y = pos_des(1);
  odom_msg.pose.pose.position.z = pos_des(2);
  
  odom_msg.pose.pose.orientation.x = rpy_des_temp(0);
  odom_msg.pose.pose.orientation.y = rpy_des_temp(1);
  odom_msg.pose.pose.orientation.z = rpy_des_temp(2);
  odom_msg.pose.pose.orientation.w = mode;
  
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = trace;
  odom_msg.twist.twist.linear.z = Thrust;
  odom_msg.twist.twist.angular.x = Torque(0);
  odom_msg.twist.twist.angular.y = Torque(1);
  odom_msg.twist.twist.angular.z = Torque(2);
  Des_odom_pub.publish( &odom_msg );
}

void ros_publish_UKF_state_odom() {

  odom_msg.pose.pose.position.x = VIO_pos(0);
  odom_msg.pose.pose.position.y = VIO_pos(1);
  odom_msg.pose.pose.position.z = VIO_pos(2);
  odom_msg.pose.pose.orientation.x = rpy_UKF(0);
  odom_msg.pose.pose.orientation.y = rpy_UKF(1);
  odom_msg.pose.pose.orientation.z = rpy_UKF(2);
  odom_msg.pose.pose.orientation.w = 1.0 / dt;
  odom_msg.twist.twist.linear.x = body_vel.X();
  odom_msg.twist.twist.linear.y = body_vel.Y();
  odom_msg.twist.twist.linear.z = body_vel.Z();
  odom_msg.twist.twist.angular.x = omega.X();
  odom_msg.twist.twist.angular.y = omega.Y();
  odom_msg.twist.twist.angular.z = omega.Z();
  UKF_state_odom_pub.publish( &odom_msg );
}

void ros_publish_UKF_param() {

  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = deg;
  odom_msg.pose.pose.orientation.y = volt;
  odom_msg.pose.pose.orientation.z = amp;
  odom_msg.pose.pose.orientation.w = rpm;
  odom_msg.twist.twist.linear.x = wind_vel.X();
  odom_msg.twist.twist.linear.y = wind_vel.Y();
  odom_msg.twist.twist.linear.z = wind_vel.Z();
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = 0;
  UKF_param_odom_pub.publish( &odom_msg );
}

void ros_publish_Madg_odom() {

  odom_msg.pose.pose.position.x = VIO_pos(0);
  odom_msg.pose.pose.position.y = VIO_pos(1);
  odom_msg.pose.pose.position.z = VIO_pos(2);
  odom_msg.pose.pose.orientation.x = rpy(0);
  odom_msg.pose.pose.orientation.y = rpy(1);
  odom_msg.pose.pose.orientation.z = rpy(2);
  odom_msg.pose.pose.orientation.w = 1.0 / dt;
  odom_msg.twist.twist.linear.x = VIO_vel(0);
  odom_msg.twist.twist.linear.y = VIO_vel(1);
  odom_msg.twist.twist.linear.z = VIO_vel(2);
  odom_msg.twist.twist.angular.x = Gyro.X();
  odom_msg.twist.twist.angular.y = Gyro.Y();
  odom_msg.twist.twist.angular.z = Gyro.Z();
  Madg_odom_pub.publish( &odom_msg );
}

void ros_publish_imu() {

  imu_msg.linear_acceleration.x = Acc.X();
  imu_msg.linear_acceleration.y = Acc.Y();
  imu_msg.linear_acceleration.z = Acc.Z();
  imu_msg.angular_velocity.x = Gyro.X();
  imu_msg.angular_velocity.y = Gyro.Y();
  imu_msg.angular_velocity.z = Gyro.Z();
  imu_pub.publish( &imu_msg );
}

void ros_publish_debug() {
  Point rpy_des_temp = EulerAnglesFrom(R_des);
  const uint16_t debug_dim = 1+12+4+12+4+4+6+4+5+3+3+3;
  float data[] = {current_time*micros2secs,
    pos_des(0),pos_des(1),pos_des(2),
    v_des(0),v_des(1),v_des(2),
    rpy_des_temp(0),rpy_des_temp(1),rpy_des_temp(2),
    omega_des(0),omega_des(1),omega_des(2),
    Motor_RPM_Des(0),Motor_RPM_Des(1),Motor_RPM_Des(2),Motor_RPM_Des(3),
    VIO_pos(0),VIO_pos(1),VIO_pos(2),
    body_vel(0),body_vel(1),body_vel(2),
    rpy_UKF(0),rpy_UKF(1),rpy_UKF(2),
    omega(0),omega(1),omega(2),
    Motor_RPM(0),Motor_RPM(1),Motor_RPM(2),Motor_RPM(3),
    Thrust,Torque(0),Torque(1),Torque(2),
    Acc(0),Acc(1),Acc(2),Gyro(0),Gyro(1),Gyro(2),
    VIO_vel(0),VIO_vel(1),VIO_vel(2),VIO_Euler(2),
    volt_lp,volt,amp,mah,deg,
    wind_vel(0),wind_vel(1),wind_vel(2),
    rpy(0),rpy(1),rpy(2),
    trace, mode, 1.0 / dt
  };

  array_msg.data = data;
  //  array_msg.layout.dim_length = 1;
  array_msg.data_length = debug_dim;

//  array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//  array_msg.layout.dim[0].label = "telemetry";
//  array_msg.layout.dim[0].size = debug_dim;
//  array_msg.layout.dim[0].stride = debug_dim;
//  array_msg.data.clear();
//  array_msg.data.insert(array_msg.data.end(), data.begin(), data.end());
  debug_pub.publish( &array_msg );
}

#endif
