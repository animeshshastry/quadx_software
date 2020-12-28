//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.2

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6,  rising_edge_start_7,  rising_edge_start_8;
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw, channel_7_raw, channel_8_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1
unsigned long channel_7_fs = 2000; //aux1
unsigned long channel_8_fs = 2000; //aux1

void radioSetup() {
  //PPM Receiver
  //Declare interrupt pin
  pinMode(PPM_Pin, INPUT_PULLUP);
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines
  unsigned long returnPWM;

  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  else if (ch_num == 7) {
    returnPWM = channel_7_raw;
  }
  else if (ch_num == 8) {
    returnPWM = channel_8_raw;
  }

  return returnPWM;
}



//========================================================================================================================//



//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig == 1) { //only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();


    if (dt_ppm > 5000) { //waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }

    if (ppm_counter == 1) { //first pulse
      channel_1_raw = dt_ppm;
    }

    if (ppm_counter == 2) { //second pulse
      channel_2_raw = dt_ppm;
    }

    if (ppm_counter == 3) { //third pulse
      channel_3_raw = dt_ppm;
    }

    if (ppm_counter == 4) { //fourth pulse
      channel_4_raw = dt_ppm;
    }

    if (ppm_counter == 5) { //fifth pulse
      channel_5_raw = dt_ppm;
    }

    if (ppm_counter == 6) { //sixth pulse
      channel_6_raw = dt_ppm;
    }

    if (ppm_counter == 7) { //sixth pulse
      channel_7_raw = dt_ppm;
    }

    if (ppm_counter == 8) { //sixth pulse
      channel_8_raw = dt_ppm;
    }

    ppm_counter = ppm_counter + 1;
  }
}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio

  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);
  channel_7_pwm = getRadioPWM(7);
  channel_8_pwm = getRadioPWM(8);

  //Low-pass the critical commands and update previous values
  float b = 0.2; //lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
     Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
     the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
     connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
     channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
     your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
  */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
  int check7 = 0;
  int check8 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;
  if (channel_7_pwm > maxVal || channel_7_pwm < minVal) check7 = 1;
  if (channel_8_pwm > maxVal || channel_8_pwm < minVal) check8 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6 + check7 + check8) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
    channel_7_pwm = channel_7_fs;
    channel_8_pwm = channel_8_fs;
  }
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
     Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
     RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
     roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
     (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
     yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
  */
  thro_des = (channel_1_pwm - 1000.0) / 1000.0; //between 0 and 1
  roll_des = (channel_2_pwm - 1500.0) / 500.0; //between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0) / 500.0; //between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0) / 500.0; //between -1 and 1
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll; //between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; //between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw; //between -maxYaw and +maxYaw
}

void printRadioData(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" CH1: "));
    SERIAL_PORT.print(channel_1_pwm);
    SERIAL_PORT.print(F(" CH2: "));
    SERIAL_PORT.print(channel_2_pwm);
    SERIAL_PORT.print(F(" CH3: "));
    SERIAL_PORT.print(channel_3_pwm);
    SERIAL_PORT.print(F(" CH4: "));
    SERIAL_PORT.print(channel_4_pwm);
    SERIAL_PORT.print(F(" CH5: "));
    SERIAL_PORT.print(channel_5_pwm);
    SERIAL_PORT.print(F(" CH6: "));
    SERIAL_PORT.print(channel_6_pwm);
    SERIAL_PORT.print(F(" CH7: "));
    SERIAL_PORT.print(channel_7_pwm);
    SERIAL_PORT.print(F(" CH8: "));
    SERIAL_PORT.println(channel_8_pwm);
  }
}

void printDesiredState(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F("thro_des: "));
    SERIAL_PORT.print(thro_des);
    SERIAL_PORT.print(F(" roll_des: "));
    SERIAL_PORT.print(roll_des);
    SERIAL_PORT.print(F(" pitch_des: "));
    SERIAL_PORT.print(pitch_des);
    SERIAL_PORT.print(F(" yaw_des: "));
    SERIAL_PORT.println(yaw_des);
  }
}
