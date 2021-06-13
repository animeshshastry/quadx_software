void imuSetup() {
  //Start IMU
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
//  WIRE_PORT.setClock(1000000);
  bool initialized = false;
  while ( !initialized ) {
    IMU.begin( WIRE_PORT, 1 );

//    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
//    SERIAL_PORT.println( IMU.statusString() );
    if ( IMU.status != ICM_20948_Stat_Ok ) {
//      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    } else {
      initialized = true;
    }
  }

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
//  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  IMU.swReset( );
  if ( IMU.status != ICM_20948_Stat_Ok) {
//    SERIAL_PORT.print(F("Software Reset returned: "));
//    SERIAL_PORT.println(IMU.statusString());
  }
  delay(250);

  // Now wake the sensor up
  IMU.sleep( false );
  IMU.lowPower( false );

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  IMU.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );
  if ( IMU.status != ICM_20948_Stat_Ok) {
//    SERIAL_PORT.print(F("setSampleMode returned: "));
//    SERIAL_PORT.println(IMU.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  // gpm2
  // gpm4
  // gpm8
  // gpm16

  myFSS.g = dps500;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  // dps250
  // dps500
  // dps1000
  // dps2000

  IMU.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
  if ( IMU.status != ICM_20948_Stat_Ok) {
//    SERIAL_PORT.print(F("setFullScale returned: "));
//    SERIAL_PORT.println(IMU.statusString());
  }


  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d5bw7_n8bw3;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
  // acc_d111bw4_n136bw
  // acc_d50bw4_n68bw8
  // acc_d23bw9_n34bw4
  // acc_d11bw5_n17bw
  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d5bw7_n8bw9;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
  // gyr_d196bw6_n229bw8
  // gyr_d151bw8_n187bw6
  // gyr_d119bw5_n154bw3
  // gyr_d51bw2_n73bw3
  // gyr_d23bw9_n35bw9
  // gyr_d11bw6_n17bw8
  // gyr_d5bw7_n8bw9
  // gyr_d361bw4_n376bw5

  IMU.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if ( IMU.status != ICM_20948_Stat_Ok) {
//    SERIAL_PORT.print(F("setDLPcfg returned: "));
//    SERIAL_PORT.println(IMU.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = IMU.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat = IMU.enableDLPF( ICM_20948_Internal_Gyr, true );
//  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(IMU.statusString(accDLPEnableStat));
//  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(IMU.statusString(gyrDLPEnableStat));

//  SERIAL_PORT.println();
//  SERIAL_PORT.println(F("Configuration complete!"));
}

void getIMUdata() {
  if ( IMU.dataReady() ) {
    IMU.getAGMT();                // The values are only updated when you call 'getAGMT'

    //Accelerometer
    Acc.X() = IMU.accX() / ACCEL_SCALE_FACTOR; //G's
    Acc.Y() = IMU.accY() / ACCEL_SCALE_FACTOR;
    Acc.Z() = IMU.accZ() / ACCEL_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    Acc.X() = Acc.X() - AccBias.X();
    Acc.Y() = Acc.Y() - AccBias.Y();
    Acc.Z() = Acc.Z() - AccBias.Z();

    //Gyro
    Gyro.X() = IMU.gyrX() * deg2rad; //rad/sec
    Gyro.Y() = IMU.gyrY() * deg2rad;
    Gyro.Z() = IMU.gyrZ() * deg2rad;
    //Correct the outputs with the calculated error values
    Gyro.X() = Gyro.X() - GyroBias.X();
    Gyro.Y() = Gyro.Y() - GyroBias.Y();
    Gyro.Z() = Gyro.Z() - GyroBias.Z();

    //Magnetometer
    Mag.X() = IMU.magX() / 6.0; //uT
    Mag.Y() = IMU.magY() / 6.0;
    Mag.Z() = IMU.magZ() / 6.0;
    //Correct the outputs with the calculated error values
    Mag.X() = (Mag.X() - MagErr.X()) * MagScale.X();
    Mag.Y() = (Mag.Y() - MagErr.Y()) * MagScale.Y();
    Mag.Z() = (Mag.Z() - MagErr.Z()) * MagScale.Z();
  }
}

void printAccMagnitude(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
#define acc_plot_scaling 1.0
    SERIAL_PORT.print(F(" |acc|: "));
    SERIAL_PORT.print(Acc.Magnitude());
  }
}

void printAccData(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
#define acc_plot_scaling 100.0
    SERIAL_PORT.print(F(" accX: "));
    SERIAL_PORT.print(acc_plot_scaling * Acc.X());
    SERIAL_PORT.print(F(" accY: "));
    SERIAL_PORT.print(acc_plot_scaling * Acc.Y());
    SERIAL_PORT.print(F(" accZ: "));
    SERIAL_PORT.print(acc_plot_scaling * (Acc.Z()-9.81));
  }
}

void printGyroData(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
#define gyro_plot_scaling rad2deg
//    SERIAL_PORT.print(F(" gyroX: "));
//    SERIAL_PORT.print(gyro_plot_scaling * Gyro.X());
    SERIAL_PORT.print(F(" gyroY: "));
    SERIAL_PORT.print(gyro_plot_scaling * Gyro.Y());
//    SERIAL_PORT.print(F(" gyroZ: "));
//    SERIAL_PORT.print(gyro_plot_scaling * Gyro.Z());
  }
}

void printMagData(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
#define mag_plot_scaling 1000000.0
    SERIAL_PORT.print(F(" magX: "));
    SERIAL_PORT.print(mag_plot_scaling * Mag.X());
    SERIAL_PORT.print(F(" magY: "));
    SERIAL_PORT.print(mag_plot_scaling * Mag.Y());
    SERIAL_PORT.print(F(" magZ: "));
    SERIAL_PORT.print(mag_plot_scaling * Mag.Z());
  }
}

void printIMUdata(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    //printRawAGMT( IMU.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT( IMU.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units

#define acc_plot_scaling 100.0
#define gyro_plot_scaling 100.0

    SERIAL_PORT.print(F(" accX: "));
    SERIAL_PORT.print(acc_plot_scaling * Acc.X());
    SERIAL_PORT.print(F(" accX: "));
    SERIAL_PORT.print(acc_plot_scaling * Acc.Y());
    SERIAL_PORT.print(F(" accZ: "));
    SERIAL_PORT.print(acc_plot_scaling * Acc.Z());
//    SERIAL_PORT.print(F(" gyroX: "));
//    SERIAL_PORT.print(gyro_plot_scaling * Gyro.X());
//    SERIAL_PORT.print(F(" gyroY: "));
//    SERIAL_PORT.print(gyro_plot_scaling * Gyro.Y());
//    SERIAL_PORT.print(F(" gyroZ: "));
//    SERIAL_PORT.println(gyro_plot_scaling * Gyro.Z());

    SERIAL_PORT.print(F(" raw accX: "));
    SERIAL_PORT.print(acc_plot_scaling * IMU.agmt.acc.axes.x / 16384.0);
    SERIAL_PORT.print(F(" raw accX: "));
    SERIAL_PORT.print(acc_plot_scaling * IMU.agmt.acc.axes.y / 16384.0);
    SERIAL_PORT.print(F(" raw accZ: "));
    SERIAL_PORT.println(acc_plot_scaling * (IMU.agmt.acc.axes.z / 16384.0));
    //    SERIAL_PORT.print(F(" raw gyroX: "));
    //    SERIAL_PORT.print(GyroX);
    //    SERIAL_PORT.print(F(" raw gyroY: "));
    //    SERIAL_PORT.print(GyroY);
    //    SERIAL_PORT.print(F(" raw gyroZ: "));
    //    SERIAL_PORT.println(GyroZ);

  }
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    SERIAL_PORT.print("-");
  } else {
    SERIAL_PORT.print(" ");
  }
  for ( uint8_t indi = 0; indi < leading; indi++ ) {
    uint32_t tenpow = 0;
    if ( indi < (leading - 1) ) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if ( aval < tenpow) {
      SERIAL_PORT.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  } else {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt) {
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( IMU.accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( IMU.accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( IMU.accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( IMU.gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( IMU.gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( IMU.gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat( IMU.magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( IMU.magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( IMU.magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat( IMU.temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printPaddedInt16b( int16_t val ) {
  if (val > 0) {
    SERIAL_PORT.print(" ");
    if (val < 10000) {
      SERIAL_PORT.print("0");
    }
    if (val < 1000 ) {
      SERIAL_PORT.print("0");
    }
    if (val < 100  ) {
      SERIAL_PORT.print("0");
    }
    if (val < 10   ) {
      SERIAL_PORT.print("0");
    }
  } else {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000) {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000 ) {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100  ) {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10   ) {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt) {
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
