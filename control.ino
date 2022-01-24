//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry
//========================================================================================================================//

Point Force_Aero, Torque_Aero;
Point Rx, Ry, Rz, xb, yb, zb, xc;
Matrix<3, 3> R_des_T, R_T;
Matrix<3, 1> R_err, omega_err, R_err_int, omega_err_int;
Matrix<3, 1> pos_err, vel_err, pos_err_int, vel_err_int;

void controlSetup() {
  R_err_int.Fill(0.0);
  omega_err_int.Fill(0.0);
  pos_err_int.Fill(0.0);
  vel_err_int.Fill(0.0);
  pos_des.Fill(0.0);
  reset_origin_flag = true;
  reset_yaw_flag = true;
}

void controlOmega() {

  Force_Aero = Aerodynamics(body_vel);
  Torque_Aero = rCP.CrossProduct(Force_Aero);

  //  Point u = (v_des - body_vel) * K_v + gravity + Force_Aero * (1.0 / mass);
  //  Thrust = mass * u(2);
  Thrust = 4 * mass * (thro_des - 0.25) * gravity(2);

  omega_err = omega - omega_des;
  //    Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * omega_des);
  Torque = (Inertia * (- omega_err * K_omega) + cross_term - omega_transf_term + Torque_Aero);
  Torque(2) = yaw_ctrl_scale * Torque(2);

  //  Serial << "Torque: " << Torque << '\n';
}

void controlSO3() {
  Force_Aero = Aerodynamics(body_vel);
  Torque_Aero = rCP.CrossProduct(Force_Aero);

  vel_err = v_des;
  Point u = vel_err * K_v + gravity + Force_Aero * (1.0 / mass);

  Rz = R.Submatrix(Slice<0, 3>(), Slice<2, 3>());
  Thrust = mass * (Rz.DotProduct(u));
  //  Thrust = 2 * mass * (thro_des-0.25) * gravity(2);

  R_des_T = ~R_des;
  R_T = ~R;
  R_err = vee(R_des_T * R - R_T * R_des) * 0.5f;
  omega_err = omega - R_T * R_des * omega_des;
  //  Serial << "R_err: " << R_err << '\n';
  //  Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * R_T * R_des * omega_des);
  Torque = Inertia * (-R_err * K_R - omega_err * K_omega - R_err_int * K_RI) + cross_term - omega_transf_term + Torque_Aero;
  Torque(2) = yaw_ctrl_scale * Torque(2);

  R_err_int += R_err * dt;
  R_err_int(0) = constrain(R_err_int(0), -.2, .2);
  R_err_int(1) = constrain(R_err_int(1), -.2, .2);
  R_err_int(2) = constrain(R_err_int(2), -.2, .2);

  //omega_err_int(0) = 0.0;
  //omega_err_int(1) = 0.0;
  //omega_err_int(2) += omega_err(2) * dt;
  //omega_err_int(2) = constrain(omega_err_int(2), -.2, .2);

  //  Serial << "R_err_int: " << R_err_int * K_RI << '\n';
  //  Serial << "Torque: " << Torque << '\n';
}

void controlSE3_vel() {

  Force_Aero = R * Aerodynamics(body_vel);
  Torque_Aero = rCP.CrossProduct(Force_Aero);

  vel_err = v_des - R * body_vel;
  Point u = vel_err * K_v + vel_err_int * K_vI + gravity + Force_Aero * (1.0f / mass);
  u(0) = constrain(u(0), -2 * 9.81, 2 * 9.81);
  u(1) = constrain(u(1), -2 * 9.81, 2 * 9.81);
  u(2) = constrain(u(2), 0.1 * 9.81, 4 * 9.81);

  Rz = R.Submatrix(Slice<0, 3>(), Slice<2, 3>());
  Thrust = mass * (Rz.DotProduct(u));

  zb = u / u.Magnitude();
  xc(0) = cos(rpy_des(2)); xc(1) = sin(rpy_des(2)); xc(2) = 0;
  // xc(0) = 1.0; xc(1) = 0.0; xc(2) = 0.0;
  yb = zb.CrossProduct(xc);
  yb = yb / yb.Magnitude();
  xb = yb.CrossProduct(zb);
  R_des.FromOrthoVectors(xb, yb, zb);

  //rpy_des = EulerAnglesFrom(R_des);
  //rpy_des(2) = rpy_UKF(2);

  R_des_T = ~R_des;
  R_T = ~R;
  R_err = vee(R_des_T * R - R_T * R_des) * 0.5f;
  omega_err = omega - R_T * R_des * omega_des;
  //  Serial << "R_err: " << R_err << '\n';
  //  Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * R_T * R_des * omega_des);
  Torque = Inertia * (-R_err * K_R - omega_err * K_omega - R_err_int * K_RI) + cross_term - omega_transf_term + Torque_Aero;
  Torque(2) = yaw_ctrl_scale * Torque(2);

  R_err_int += R_err * dt;
  R_err_int(0) = constrain(R_err_int(0), -.2, .2);
  R_err_int(1) = constrain(R_err_int(1), -.2, .2);
  R_err_int(2) = constrain(R_err_int(2), -.2, .2);

  //omega_err_int(0) = 0.0;
  //omega_err_int(1) = 0.0;
  //omega_err_int(2) += omega_err(2) * dt;
  //omega_err_int(2) = constrain(omega_err_int(2), -.2, .2);

  vel_err_int += vel_err * dt;
  vel_err_int(0) = constrain(vel_err_int(0), -.5, .5);
  vel_err_int(1) = constrain(vel_err_int(1), -.5, .5);
  vel_err_int(2) = constrain(vel_err_int(2), -.5, .5);

  //  Serial << "Torque: " << Torque << '\n';
}

void controlSE3_pos(int aero_flag) {

#if (wind_est)
#if (wind_in_body_frame)
  Force_Aero = Aerodynamics(body_vel - wind_vel);
#else
//  Force_Aero = Aerodynamics(body_vel - (~R) * wind_vel);
  Point Force_Aero = Aerodynamics2(body_vel+omega.CrossProduct(rCP),wind_vel);
#endif
#else
  Force_Aero = Aerodynamics(body_vel);
#endif

  Torque_Aero = rCP.CrossProduct(Force_Aero) * 0.0;

  pos_err = pos_des - VIO_pos;
  vel_err = v_des - R * body_vel;
  Point u = acc_des + pos_err * K_p + vel_err * K_v + pos_err_int * K_pI + gravity - R * Force_Aero * (1.0f / mass) * aero_flag;
  u(0) = constrain(u(0), -2 * gravity(2), 2 * gravity(2));
  u(1) = constrain(u(1), -2 * gravity(2), 2 * gravity(2));
  u(2) = constrain(u(2), 0.1 * gravity(2), 4 * gravity(2));

  Rz = R.Submatrix(Slice<0, 3>(), Slice<2, 3>());
  Thrust = mass * (Rz.DotProduct(u));

  zb = u / u.Magnitude();
  xc(0) = cos(rpy_des(2)); xc(1) = sin(rpy_des(2)); xc(2) = 0;
  // xc(0) = 1.0; xc(1) = 0.0; xc(2) = 0.0;
  yb = zb.CrossProduct(xc);
  yb = yb / yb.Magnitude();
  xb = yb.CrossProduct(zb);
  R_des.FromOrthoVectors(xb, yb, zb);

  //rpy_des = EulerAnglesFrom(R_des);

  R_des_T = ~R_des;
  R_T = ~R;
  R_err = vee(R_des_T * R - R_T * R_des) * 0.5f;
  omega_err = omega - R_T * R_des * omega_des;
  //  Serial << "R_err: " << R_err << '\n';
  //  Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * R_T * R_des * omega_des);
  Torque = Inertia * (-R_err * K_R - omega_err * K_omega - R_err_int * K_RI) + cross_term - omega_transf_term;
//  Torque = Inertia * (-R_err * K_R - omega_err * K_omega - R_err_int * K_RI) + cross_term - omega_transf_term - Torque_Aero * aero_flag;
  Torque(2) = yaw_ctrl_scale * Torque(2);

  R_err_int += R_err * dt;
  R_err_int(0) = constrain(R_err_int(0), -.3, .3);
  R_err_int(1) = constrain(R_err_int(1), -.3, .3);
  R_err_int(2) = constrain(R_err_int(2), -.3, .3);

  //omega_err_int(0) = 0.0;
  //omega_err_int(1) = 0.0;
  //omega_err_int(2) += omega_err(2) * dt;
  //omega_err_int(2) = constrain(omega_err_int(2), -.2, .2);

  pos_err_int += pos_err * dt;
  pos_err_int(0) = constrain(pos_err_int(0), -.5, .5);
  pos_err_int(1) = constrain(pos_err_int(1), -.5, .5);
  pos_err_int(2) = constrain(pos_err_int(2), -.5, .5);

  //  Serial << "Torque: " << Torque << '\n';
}

void printControlOutput(int print_rate) {
#define print_scale 10.0
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" Thrust (N): "));
    SERIAL_PORT.print(Thrust);
    SERIAL_PORT.print(F(" Mx (N-m): "));
    SERIAL_PORT.print(Torque(0)*print_scale);
    SERIAL_PORT.print(F(" My (N-m): "));
    SERIAL_PORT.print(Torque(1)*print_scale);
    SERIAL_PORT.print(F(" Mz (N-m): "));
    SERIAL_PORT.println(Torque(2)*print_scale);
  }
}
