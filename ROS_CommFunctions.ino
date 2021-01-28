//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry

//========================================================================================================================//

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_raw", &imu_msg);
nav_msgs::Odometry odom_msg;
ros::Publisher Madg_odom_pub("odom_Madg", &odom_msg);
ros::Publisher UKF_odom_pub("odom_UKF", &odom_msg);
//ros::Subscriber<nav_msgs::Odometry> odom_sub("/camera/odom/sample", vio_callback );
//ros::Subscriber<nav_msgs::Odometry> odom_sub("/VIO_odom", vio_callback );
ros::Subscriber<geometry_msgs::Point> pos_sub("/VIO_pos", pos_callback );
ros::Subscriber<geometry_msgs::Vector3> vel_sub("/VIO_vel", vel_callback );

void ROS_CommSetup() {
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(Madg_odom_pub);
  nh.advertise(UKF_odom_pub);
  //nh.subscribe(odom_sub);
  nh.subscribe(pos_sub);
  nh.subscribe(vel_sub);
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
  VIO_pos(0) = msg.x;
  VIO_pos(1) = msg.y;
  VIO_pos(2) = msg.z;
}

void vel_callback( const geometry_msgs::Vector3& msg) {
  VIO_vel(0) = msg.x;
  VIO_vel(1) = msg.y;
  VIO_vel(2) = msg.z;
}

void ros_publish_UKF_odom() {
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = "map";
  //odom_msg.header.stamp.sec = current_time * micros2secs;
  //odom_msg.header.stamp.nsec = (current_time * micros2secs - imu_msg.header.stamp.sec) * secs2nsecs;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = VIO_pos(0);
  odom_msg.pose.pose.position.y = VIO_pos(1);
  odom_msg.pose.pose.position.z = VIO_pos(2);
  odom_msg.pose.pose.orientation.x = rpy_UKF(0);
  odom_msg.pose.pose.orientation.y = rpy_UKF(1);
  odom_msg.pose.pose.orientation.z = rpy_UKF(2);
  odom_msg.pose.pose.orientation.w = 0.0;
  odom_msg.twist.twist.linear.x = body_vel.X();
  odom_msg.twist.twist.linear.y = body_vel.Y();
  odom_msg.twist.twist.linear.z = body_vel.Z();
  odom_msg.twist.twist.angular.x = omega.X();
  odom_msg.twist.twist.angular.y = omega.Y();
  odom_msg.twist.twist.angular.z = omega.Z();
  UKF_odom_pub.publish( &odom_msg );
}

void ros_publish_Madg_odom() {
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = "map";
  //odom_msg.header.stamp.sec = current_time * micros2secs;
  //odom_msg.header.stamp.nsec = (current_time * micros2secs - imu_msg.header.stamp.sec) * secs2nsecs;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = rpy(0);
  odom_msg.pose.pose.orientation.y = rpy(1);
  odom_msg.pose.pose.orientation.z = rpy(2);
  odom_msg.pose.pose.orientation.w = 0.0;
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = Gyro.X();
  odom_msg.twist.twist.angular.y = Gyro.Y();
  odom_msg.twist.twist.angular.z = Gyro.Z();
  Madg_odom_pub.publish( &odom_msg );
}

void ros_publish_imu() {
  imu_msg.header.frame_id = "map";
  imu_msg.header.stamp.sec = current_time * micros2secs;
  imu_msg.header.stamp.nsec = (current_time * micros2secs - imu_msg.header.stamp.sec) * secs2nsecs;
  imu_msg.header.stamp = nh.now();
  imu_msg.linear_acceleration.x = Acc.X();
  imu_msg.linear_acceleration.y = Acc.Y();
  imu_msg.linear_acceleration.z = Acc.Z();
  imu_msg.angular_velocity.x = Gyro.X();
  imu_msg.angular_velocity.y = Gyro.Y();
  imu_msg.angular_velocity.z = Gyro.Z();
  imu_pub.publish( &imu_msg );
}
