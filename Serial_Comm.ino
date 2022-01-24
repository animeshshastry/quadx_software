//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry

//========================================================================================================================//

#define receive_data_dim 9

char inputString[400] = "";         // a String to hold incoming data
int idx = 0;
char prev_char = 0;
char prev_prev_char = 0;;
char new_char = 0;
String value = "";
int comma_received = 0;
float received_data[receive_data_dim];
long sum_of_digits = 0;
long checksum = 0, n_data_received = 0;

bool recvInProgress = false;
bool newData = false;

uint8_t vrbl_trnsmt_ctr = 0;

float dt_sc;
unsigned long current_time_sc, prev_time_sc;

void read_Serial() {

  while (SERIAL_PORT.available() > 0 && newData == false) {
    // get the new byte:
    prev_prev_char = prev_char;
    prev_char = new_char;
    new_char = (char)SERIAL_PORT.read();
    //Serial.println(new_char);
    if (recvInProgress == true) {
      if (comma_received < receive_data_dim) {
        if (new_char == '-' && prev_char == ',') {
          value = new_char;
        }
        else if ((new_char >= '0' && new_char <= '9') || new_char == '.') {
          value += new_char;
          if (new_char != '.') {
            sum_of_digits += new_char - '0';
          }
        }
        else if (new_char == ',' && value != "") {
          // new command received
          received_data[comma_received] = value.toFloat();
          // Serial.println(comma_received);
          value = "";
          comma_received++;
        }
        else {
          // wrong character
          recvInProgress = false;
        }
      }
      else {
        if (new_char >= '0' && new_char <= '9') {
          value += new_char;
        }
        else if (new_char == '\n') {
          checksum = value.toInt();
          if (checksum == sum_of_digits) {
            newData = true;
            n_data_received++;
          }
          recvInProgress = false;
        }
        else {
          recvInProgress = false;
        }
      }
    }
    else if (prev_prev_char == 'H' && prev_char == 'i' && new_char == ',') {
      recvInProgress = true;
      comma_received = 0;
      value = "";
      sum_of_digits = 0;
    }
  }

  if (newData) {
    VIO_pos(0) = received_data[0];
    VIO_pos(1) = received_data[1];
    VIO_pos(2) = received_data[2];
    VIO_vel(0) = received_data[3];
    VIO_vel(1) = received_data[4];
    VIO_vel(2) = received_data[5];
    VIO_Euler(0) = received_data[6];
    VIO_Euler(1) = received_data[7];
    VIO_Euler(2) = received_data[8];
    prev_time_sc = current_time_sc;
    current_time_sc = micros();
    dt_sc = (current_time_sc - prev_time_sc) * micros2secs;
    Serial.print(F("Read Freq: "));
    Serial.println(1/dt_sc);
    newData = false;
  }
}

void send_IMU_data(){

  const int dim = 6;
  Matrix<dim, 1> Data_array = Acc && Gyro;
  
  for (int i = 0; i < dim; i++) {
    String data = "Hi,prm,";
    if (i < 10) {
      data = data + "I" + "0" + String(i) + "," + String(Data_array(i), 5);
    }
    else {
      data = data + "I" + String(i) + "," + String(Data_array(i), 5);
    }
    data = data + "," + String(data.length()) + "\n";
    SERIAL_PORT.print(data);
    //delay(1);
  }
}

void send_Madg_data(){
  
  const int dim = 12;
  Matrix<dim, 1> Data_array = VIO_pos && rpy && VIO_vel && Gyro;
  
  for (int i = 0; i < dim; i++) {
    String data = "Hi,prm,";
    if (i < 10) {
      data = data + "M" + "0" + String(i) + "," + String(Data_array(i), 5);
    }
    else {
      data = data + "M" + String(i) + "," + String(Data_array(i), 5);
    }
    data = data + "," + String(data.length()) + "\n";
    SERIAL_PORT.print(data);
    //delay(1);
  }
}

void send_UKF_state_data(){
  
  const int dim = 12;
  Matrix<dim, 1> Data_array = VIO_pos && rpy_UKF && body_vel && omega;
  
  for (int i = 0; i < dim; i++) {
    String data = "Hi,prm,";
    if (i < 10) {
      data = data + "U" + "0" + String(i) + "," + String(Data_array(i), 5);
    }
    else {
      data = data + "U" + String(i) + "," + String(Data_array(i), 5);
    }
    data = data + "," + String(data.length()) + "\n";
    SERIAL_PORT.print(data);
    //delay(1);
  }
}

void send_UKF_param_data(){
  
  const int dim = 12;
  Matrix<dim, 1> Data_array = VIO_pos && rpy_UKF && wind_vel && omega;
  
  for (int i = 0; i < dim; i++) {
    String data = "Hi,prm,";
    if (i < 10) {
      data = data + "P" + "0" + String(i) + "," + String(Data_array(i), 5);
    }
    else {
      data = data + "P" + String(i) + "," + String(Data_array(i), 5);
    }
    data = data + "," + String(data.length()) + "\n";
    SERIAL_PORT.print(data);
    //delay(1);
  }
}

void send_Des_data(){

  Point rpy_des_temp = EulerAnglesFrom(R_des);
  Matrix<3, 1> ctrl_temp = {0.0, trace, Thrust};
  
  const int dim = 12;
  Matrix<dim, 1> Data_array = pos_des && rpy_des_temp && ctrl_temp && Torque;
  
  for (int i = 0; i < dim; i++) {
    String data = "Hi,prm,";
    if (i < 10) {
      data = data + "D" + "0" + String(i) + "," + String(Data_array(i), 5);
    }
    else {
      data = data + "D" + String(i) + "," + String(Data_array(i), 5);
    }
    data = data + "," + String(data.length()) + "\n";
    SERIAL_PORT.print(data);
    //delay(1);
  }
}
