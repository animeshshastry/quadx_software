ArrayMatrix<4, 1, double> RPM_err, RPM_err_int;

void motorSetup() {
  init_MixerX();
  setup_ESCs();
}

void setup_ESCs()
{
  // initialize DSHOT
  DSHOT_init( N_ESC );

  // ARM ESC( though I think this just selects DSHOT)
  for (uint8_t i = 0; i < N_ESC; i++)
  {
    cmd[i] = 0;
    tlm[i] = 0;
  }
  DSHOT_send( cmd, tlm );
//  delayMicroseconds(10*DSHOT_DELAY);
  delay(1000); // for the voice DHSOT detected (KISS ESC) !!
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
                       -kmf, kmf, -kmf,  kmf
                     };
  MIX_INV = A_f.Inverse();
}

void Mixer() {
  // x config mixer
  //  Motor_RPM_Des(0) = (0.25 * T) / k_f - (0.25 * M[3]) / k_m + (0.25 * M[1]) / (L * k_f) - (0.25 * M[2]) / (L * k_f);
  //  Motor_RPM_Des(1) = (0.25 * T) / k_f + (0.25 * M[3]) / k_m + (0.25 * M[1]) / (L * k_f) + (0.25 * M[2]) / (L * k_f);
  //  Motor_RPM_Des(2) = (0.25 * T) / k_f - (0.25 * M[3]) / k_m - (0.25 * M[1]) / (L * k_f) + (0.25 * M[2]) / (L * k_f);
  //  Motor_RPM_Des(3) = (0.25 * T) / k_f + (0.25 * M[3]) / k_m - (0.25 * M[1]) / (L * k_f) - (0.25 * M[2]) / (L * k_f);

  Matrix<4, 1> U = {Thrust, Torque(0), Torque(1), Torque(2)};
  //  Motor_RPM_Des = MIX_INV * U;
  Motor_Thrust = MIX_INV * U;

  for (uint8_t i=0; i<4; i++){
//    Motor_RPM_Des(i) = QuadraticSolve(kf2,kf1,(-Motor_Thrust(i)));
//    Motor_RPM_Des(i) = sqrt(Motor_Thrust(i)/k_f);
    Motor_RPM_Des(i) = sign(Motor_Thrust(i))*sqrt(abs(Motor_Thrust(i))/k_f);
    Motor_RPM_Des(i) = constrain(Motor_RPM_Des(i), 0.0, MAX_RPM);
  }
}

void send_motor_cmds() {
  if (!disarmed)
  {
    // DSHOT cmds
    //  cmd[0] = Motor_RPM_Des(0) * 1999 + 48;
    //  cmd[1] = Motor_RPM_Des(1) * 1999 + 48;
    //  cmd[3] = Motor_RPM_Des(2) * 1999 + 48;
    //  cmd[5] = Motor_RPM_Des(3) * 1999 + 48;
    
//    cmd[0] = 1*Motor_Thrust(0) * MTS + MTC;
//    cmd[1] = 1*Motor_Thrust(1) * MTS + MTC;
//    cmd[3] = 1*Motor_Thrust(2) * MTS + MTC;
//    cmd[5] = 1*Motor_Thrust(3) * MTS + MTC;

#if (!VOLT_CORR)
  float VCF = 1.0;
#else
  float VCF = (volt_lp/15.0);
  VCF = constrain(VCF, 0.7, 2);
//  Serial.println(VCF);
//  VCF *= VCF;
#endif

#if (!CLOSE_RPM_LOOP)
    cmd[0] = MT2 * Motor_RPM_Des(0)*Motor_RPM_Des(0) + MT1 * Motor_RPM_Des(0) + MT0;
    cmd[1] = MT2 * Motor_RPM_Des(1)*Motor_RPM_Des(1) + MT1 * Motor_RPM_Des(1) + MT0;
    cmd[3] = MT2 * Motor_RPM_Des(2)*Motor_RPM_Des(2) + MT1 * Motor_RPM_Des(2) + MT0;
    cmd[5] = MT2 * Motor_RPM_Des(3)*Motor_RPM_Des(3) + MT1 * Motor_RPM_Des(3) + MT0;
//  cmd[0] = MT1_2 * Motor_RPM_Des(0) + MT0_2;
//  cmd[1] = MT1_2 * Motor_RPM_Des(1) + MT0_2;
//  cmd[3] = MT1_2 * Motor_RPM_Des(2) + MT0_2;
//  cmd[5] = MT1_2 * Motor_RPM_Des(3) + MT0_2;
  for ( uint8_t i = 0; i < N_ESC; i++ )  {
    cmd[i] *= 1.0/VCF;
//    cmd[i] *= cmd[i]/VCF;
  }
#else
  RPM_err = Motor_RPM_Des - Motor_RPM;
  Motor_RPM_Des_dot = RPM_err * K_RPM + RPM_err_int * K_RPM_I; 
  cmd[0] += (2.0 * MT2 * Motor_RPM(0) + MT1) * Motor_RPM_Des_dot(0) * dt;
  cmd[1] += (2.0 * MT2 * Motor_RPM(1) + MT1) * Motor_RPM_Des_dot(1) * dt;
  cmd[3] += (2.0 * MT2 * Motor_RPM(2) + MT1) * Motor_RPM_Des_dot(2) * dt;
  cmd[5] += (2.0 * MT2 * Motor_RPM(3) + MT1) * Motor_RPM_Des_dot(3) * dt;
  for ( uint8_t i = 0; i < N_ESC; i++ )  {
    cmd[i] *= 1.0/VCF;
  }
  RPM_err_int += RPM_err * dt;
  for (uint8_t i = 0; i < 4; i++){
    RPM_err_int(i) = constrain(RPM_err_int(i), -5000, 5000);
  } 
#endif

//  cmd[0] = 50;
//  cmd[1] = 50;
//  cmd[3] = 50;
//  cmd[5] = 50;
    
//    tlm[0] = true;
//    tlm[1] = false;
//    tlm[3] = false;
//    tlm[5] = true;

    cmd[0] = constrain(cmd[0], 50, 2000);
    cmd[1] = constrain(cmd[1], 50, 2000);
    cmd[3] = constrain(cmd[3], 50, 2000);
    cmd[5] = constrain(cmd[5], 50, 2000);

#if (!RPM_TLM)
    DSHOT_send( cmd, tlm );
#if (VOLT_CORR)
    tlm[0] = true;
    get_motor_telem(0,0);
#endif
#else
//    Serial.println("Sending 1 by 1");
    for (uint8_t i = 0; i < 4; i++){
      uint8_t k;
      switch (i) {
        case 0: k=0; break;
        case 1: k=1; break;
        case 2: k=3; break;
        case 3: k=5; break;
      }
      tlm[k] = true;
      DSHOT_send( cmd, tlm );
      get_motor_telem(i,k);
      delayMicroseconds(10*DSHOT_DELAY);
    }
#endif
  }
}

void get_motor_telem(int i, uint8_t k) {
  if (tlm[k])
  {
//  int idx = 0;
  uint8_t idx = 9;
  uint8_t data_array[10];

  if (ESC_TLM_PORT.available()) {
    
//    while (Serial3.available() && idx < 10) {
    while (ESC_TLM_PORT.available()) {
//    while (idx < 10) {
      uint8_t data = ESC_TLM_PORT.read();
      data_array[idx] = data;
//      Serial.print(data);
//      Serial.print(" ");
      idx++;
      if (idx==10) idx = 0;
    }
//    Serial.println();

    volt    = ( data_array[0] << 8 ) | data_array[1];
    amp     = ( data_array[2] << 8 ) | data_array[3];
    mah     = ( data_array[4] << 8 ) | data_array[5];
//    deg     = data_array[6];
//    rpm     = ( data_array[7] << 8 ) | data_array[8];
    deg     = 0;
    rpm     = data_array[6] << 16  | data_array[7] << 8 | data_array[8]; // KISS ESC gives 3 byte hex data for rpm!!!

    uint8_t crc = get_crc8(data_array, 9); // get the 8 bit CRC

    // The data is read in the next tlm request so adjust by 1
    i--;
    if (i==-1) i=3;
    volt_lp = VOLT_LP*(volt/100.0) + (1-VOLT_LP)*volt_lp;
    Motor_RPM(i) = RPM_LP*(rpm/MOTOR_MAGNETS) + (1-RPM_LP)*Motor_RPM(i);

//    if (i==0) Serial.print(rpm/MOTOR_MAGNETS);
    
//    if (i==0){
//      Serial.print(" temp: ");
//      Serial.print(deg);
//      Serial.print(" volt: ");
//      Serial.print(volt);
//      Serial.print(" current: ");
//      Serial.print(amp);
//      Serial.print(" used mah: ");
//      Serial.print(mah);
//      Serial.print(" erpm: ");
//      Serial.print(rpm);
//      Serial.print(" crc: ");
//      Serial.print(crc);
//      Serial.print(" Last ele: ");
//      Serial.print(data_array[9]);
//      Serial.println(i);
//    }
  }
  }
  tlm[k] = false;
//  while (ESC_TLM_PORT.available()) {
//    uint8_t data = ESC_TLM_PORT.read();
//  }
}

void arm_motors() {
  if (disarmed) {
    setup_ESCs();
    for ( uint8_t i = 0; i < N_ESC; i++ )  {
      cmd[i] = DSHOT_STOP_CMD;
      tlm[i] = 0;
    }
    for ( uint16_t i = 0; i < ARMING_REP; i++ )  {
      DSHOT_send( cmd, tlm );
      delayMicroseconds( 2 * DSHOT_DELAY );
    }
    disarmed = false;
    controlSetup();
    volt_lp = 16.0;
  }
}

void disarm_motors() {
  if (!disarmed) {
    for ( uint8_t i = 0; i < N_ESC; i++ )  {
      cmd[i] = DSHOT_STOP_CMD;
      tlm[i] = 0;
    }
    DSHOT_send( cmd, tlm );
    delayMicroseconds(DSHOT_DELAY);
    disarmed = true;
  }
}

void printMotorOutput() {
  SERIAL_PORT.print(F(" m0_d: "));
  SERIAL_PORT.print(Motor_RPM_Des(0));
  SERIAL_PORT.print(F(" m1_d: "));
  SERIAL_PORT.print(Motor_RPM_Des(1));
  SERIAL_PORT.print(F(" m2_d: "));
  SERIAL_PORT.print(Motor_RPM_Des(2));
  SERIAL_PORT.print(F(" m3_d: "));
  SERIAL_PORT.print(Motor_RPM_Des(3));
  SERIAL_PORT.print(F(" m0: "));
  SERIAL_PORT.print(Motor_RPM(0));
  SERIAL_PORT.print(F(" m1: "));
  SERIAL_PORT.print(Motor_RPM(1));
  SERIAL_PORT.print(F(" m2: "));
  SERIAL_PORT.print(Motor_RPM(2));
  SERIAL_PORT.print(F(" m3: "));
  SERIAL_PORT.print(Motor_RPM(3));
}

void printMotorCmdRateOutput() {
  SERIAL_PORT.print(F(" m0_des_dot: "));
  SERIAL_PORT.print(Motor_RPM_Des_dot(0));
//  SERIAL_PORT.print(F(" m1_des_dot: "));
//  SERIAL_PORT.print(Motor_RPM_Des_dot(1));
//  SERIAL_PORT.print(F(" m2_des_dot: "));
//  SERIAL_PORT.print(Motor_RPM_Des_dot(2));
//  SERIAL_PORT.print(F(" m3_des_dot: "));
//  SERIAL_PORT.print(Motor_RPM_Des_dot(3));
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


uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i = 0; i < 8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for ( i = 0; i < BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
