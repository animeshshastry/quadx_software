
void motorSetup() {
  init_MixerX();
  setup_ESCs();
}

void setup_ESCs()
{
  // initialize DSHOT
  DSHOT_init( N_ESC );

  // ARM ESC( though I think this just selects DSHOT)
  for (int i = 0; i < N_ESC; i++)
  {
    cmd[i] = 0;
    tlm[i] = 0;
  }
  DSHOT_send( cmd, tlm );
  delayMicroseconds(DSHOT_DELAY);
}

void init_MixerX() {
  /*(CCW) 0   3 (CW)
            X
    (CW)  1   2 (CCW)
  */
//  Matrix<4, 4> A_f = { k_f,      k_f,      k_f,      k_f,
//                       Ly * k_f,   Ly * k_f,   -Ly * k_f,  -Ly * k_f,
//                       -Lx * k_f,  Lx * k_f,   Lx * k_f,   -Lx * k_f,
//                       -k_m,     k_m,      -k_m,     k_m
//                     };
  Matrix<4, 4> A_f = { 1,   1,    1,    1,
                       Ly,  Ly,   -Ly,  -Ly,
                       -Lx, Lx,   Lx,   -Lx,
                       kmf, -kmf, kmf,  -kmf
                     };
  MIX_INV = A_f.Inverse();
}

void Mixer() {
  // x config mixer
  //  Motor_Speed(0) = (0.25 * T) / k_f - (0.25 * M[3]) / k_m + (0.25 * M[1]) / (L * k_f) - (0.25 * M[2]) / (L * k_f);
  //  Motor_Speed(1) = (0.25 * T) / k_f + (0.25 * M[3]) / k_m + (0.25 * M[1]) / (L * k_f) + (0.25 * M[2]) / (L * k_f);
  //  Motor_Speed(2) = (0.25 * T) / k_f - (0.25 * M[3]) / k_m - (0.25 * M[1]) / (L * k_f) + (0.25 * M[2]) / (L * k_f);
  //  Motor_Speed(3) = (0.25 * T) / k_f + (0.25 * M[3]) / k_m - (0.25 * M[1]) / (L * k_f) - (0.25 * M[2]) / (L * k_f);

  Matrix<4, 1> U = {Thrust, Torque(0), Torque(1), Torque(2)};
//  Motor_Speed = MIX_INV * U;
  Motor_Thrust = MIX_INV * U;
  
//  Motor_Speed(0) = constrain(Motor_Speed(0), 0.0f, 1.0f);
//  Motor_Speed(1) = constrain(Motor_Speed(1), 0.0f, 1.0f);
//  Motor_Speed(2) = constrain(Motor_Speed(2), 0.0f, 1.0f);
//  Motor_Speed(3) = constrain(Motor_Speed(3), 0.0f, 1.0f);
}

void send_motor_cmds() {
  if (!disarmed)
  {
      // DSHOT cmds
  //  cmd[0] = Motor_Speed(0) * 1999 + 48;
  //  cmd[1] = Motor_Speed(1) * 1999 + 48;
  //  cmd[3] = Motor_Speed(2) * 1999 + 48;
  //  cmd[5] = Motor_Speed(3) * 1999 + 48;
    cmd[0] = Motor_Thrust(0) * MTS + MTC;
    cmd[1] = Motor_Thrust(1) * MTS + MTC;
    cmd[3] = Motor_Thrust(2) * MTS + MTC;
    cmd[5] = Motor_Thrust(3) * MTS + MTC;
  
    DSHOT_send( cmd, tlm );
    delayMicroseconds(200);
    // ESC disarms if nothing is received within 1.5ms, send zero throttle(or 48) to arm again
  }
}

void arm_motors() {
  if (disarmed) {
    cmd[0] = 48;
    cmd[1] = 48;
    cmd[2] = 48;
    cmd[3] = 48;
    cmd[4] = 48;
    cmd[5] = 48;
    for (int i = 0; i < 1000; i++) {
      DSHOT_send( cmd, tlm );
      delayMicroseconds(DSHOT_DELAY);
    }
    disarmed = false;
  }
}

void disarm_motors() {
  if (!disarmed) {
    cmd[0] = 48;
    cmd[1] = 48;
    cmd[2] = 48;
    cmd[3] = 48;
    cmd[4] = 48;
    cmd[5] = 48;
    DSHOT_send( cmd, tlm );
    delayMicroseconds(DSHOT_DELAY);
    disarmed = true;
  }
}

void printMotorOutput(int print_rate) {
#define print_scale 100.0
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" m0: "));
    SERIAL_PORT.print(Motor_Speed(0)*print_scale);
    SERIAL_PORT.print(F(" m1: "));
    SERIAL_PORT.print(Motor_Speed(1)*print_scale);
    SERIAL_PORT.print(F(" m2: "));
    SERIAL_PORT.print(Motor_Speed(2)*print_scale);
    SERIAL_PORT.print(F(" m3: "));
    SERIAL_PORT.print(Motor_Speed(3)*print_scale);
  }
}

void printMotorCmdOutput(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" m0: "));
    SERIAL_PORT.print(cmd[0]);
    SERIAL_PORT.print(F(" m1: "));
    SERIAL_PORT.print(cmd[1]);
    SERIAL_PORT.print(F(" m2: "));
    SERIAL_PORT.print(cmd[3]);
    SERIAL_PORT.print(F(" m3: "));
    SERIAL_PORT.print(cmd[5]);
  }
}
