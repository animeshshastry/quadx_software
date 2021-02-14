//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry
//========================================================================================================================//

Point Force_Aero, Torque_Aero;
Point Rx, Ry, Rz, xb, yb, zb, xc;
Matrix<3, 3> R_des_T, R_T;
Matrix<3, 1> R_err, omega_err;

void controlSetup() {
}

void controlOmega() {

  Force_Aero = Aerodynamics(body_vel);
  Torque_Aero = rCP.CrossProduct(Force_Aero);

//  Point u = (v_des - body_vel) * K_v + gravity + Force_Aero * (1.0 / mass);
//  Thrust = mass * u(2);
  Thrust = 4 * mass * thro_des * gravity(2);

  omega_err = omega - omega_des;
//    Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * omega_des);
  Torque = (Inertia * (- omega_err * K_omega) + cross_term - omega_transf_term + Torque_Aero)*0.0;

  //  Serial << "Torque: " << Torque << '\n';
}

void controlSO3() {
  Force_Aero = Aerodynamics(body_vel);
  Torque_Aero = rCP.CrossProduct(Force_Aero);

  Point u = (v_des - body_vel) * K_v + gravity + Force_Aero * (1.0 / mass);

  Rz = R.Submatrix(Slice<0, 3>(), Slice<2, 3>());
  Thrust = mass * (Rz.DotProduct(u));

  R_des_T = ~R_des;
  R_T = ~R;
  R_err = vee(R_des_T * R - R_T * R_des) * 0.5f;
  omega_err = omega - R_T * R_des * omega_des;
  //  Serial << "R_err: " << R_err << '\n';
  //  Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * R_T * R_des * omega_des);
  Torque = Inertia * (-R_err * K_R - omega_err * K_omega) + cross_term - omega_transf_term + Torque_Aero;

  //  Serial << "Torque: " << Torque << '\n';
}

void controlSE3() {

  Force_Aero = Aerodynamics(body_vel);
  Torque_Aero = rCP.CrossProduct(Force_Aero);

  Point u = (pos_des - VIO_pos) * K_p + (v_des - body_vel) * K_v + gravity + Force_Aero * (1.0f / mass);

  Rz = R.Submatrix(Slice<0, 3>(), Slice<2, 3>());
  Thrust = mass * (Rz.DotProduct(u));

  zb = u / u.Magnitude();
  xc(0) = cos(rpy_des(2)); xc(1) = sin(rpy_des(2)); xc(2) = 0;
  yb = zb.CrossProduct(xc);
  yb = yb / yb.Magnitude();
  xb = yb.CrossProduct(zb);
  R_des.FromOrthoVectors(xb, yb, zb);

  R_des_T = ~R_des;
  R_T = ~R;
  R_err = vee(R_des_T * R - R_T * R_des) * 0.5f;
  omega_err = omega - R_T * R_des * omega_des;
  //  Serial << "R_err: " << R_err << '\n';
  //  Serial << "omega_err: " << omega_err << '\n';

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_transf_term = Inertia * (hat(omega) * R_T * R_des * omega_des);
  Torque = Inertia * (-R_err * K_R - omega_err * K_omega) + cross_term - omega_transf_term + Torque_Aero;

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
