#include "poshold.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>
#include <Arduino.h>

// ════════════════════════════════════════════════════════════
//  CONFIG GPS UART
// ════════════════════════════════════════════════════════════
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21

/*
  uBlox M10 config commands (gửi 1 lần qua Serial hoặc u-center):
  Set 10 Hz : B5 62 06 8A 0A 00 00 07 00 00 01 00 21 30 64 00 57 EF
  En GGA+RMC: B5 62 06 8A 22 00 00 07 00 00 BB 00 91 20 01
              AC 00 91 20 01 CA 00 91 20 00 C0 00 91 20 00
              C5 00 91 20 00 B1 00 91 20 00 48 26
*/

static TinyGPSPlus gps;
static HardwareSerial SerialGPS(1);

uint8_t flight_mode;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
float hdop = 10.0f, gpsHeading;
uint8_t satellites = 0;

int32_t lat_gps_actual, lon_gps_actual;
int32_t lat_gps_previous, lon_gps_previous;
int32_t l_lat_gps, l_lon_gps;
int32_t l_lat_waypoint, l_lon_waypoint;
uint8_t gps_add_counter = 0, new_gps_data_counter = 0;
uint8_t new_gps_data_available;
float lat_gps_loop_add = 0, lon_gps_loop_add = 0;
uint8_t waypoint_set;
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
float lat_gps_add = 0, lon_gps_add = 0;
uint8_t gps_rotating_mem_location;
float gps_adjust_east, gps_pitch_pid_adjust, gps_adjust_north, gps_roll_pid_adjust;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
uint8_t latitude_north, longitude_east;

float gps_p_gain = 2.0, gps_d_gain = 0.02;


bool checkGPSQuality() {
  if (hdop > 2.0f) return false;     // HDOP cao = nhiễu nhiều
  if (satellites < 8) return false;  // ít vệ tinh = không tin
  return true;
}
void gpsSetup() {
  SerialGPS.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void readGPS(float yaw) {
  if (gps_add_counter > 0) gps_add_counter--;

  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // Kiểm tra nếu có dữ liệu vị trí mới và hợp lệ
  if (gps.location.isUpdated() && gps.location.isValid()) {

    // Lấy tọa độ thực tế (nhân 10^7 để khớp với định dạng số nguyên của code gốc)
    lat_gps_actual = (int32_t)(gps.location.lat() * 1000000);
    lon_gps_actual = (int32_t)(gps.location.lng() * 1000000);
    satellites = (uint8_t)gps.satellites.value();
    hdop = (float)gps.hdop.hdop();
    gpsHeading = (float)gps.course.deg();
    latitude_north = (gps.location.rawLat().negative) ? 0 : 1;
    longitude_east = (gps.location.rawLng().negative) ? 0 : 1;

    // gps_debug();
    if (hdop <= 1.5 && satellites >= 8) {
      // --- LOGIC GIỮ VỊ TRÍ (GIỮ NGUYÊN TỪ CODE GỐC) ---
      if (lat_gps_previous == 0 && lon_gps_previous == 0) {
        lat_gps_previous = lat_gps_actual;
        lon_gps_previous = lon_gps_actual;
      }
      // Tính toán bù suy diễn (Simulation) cho vòng lặp 250Hz
      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 13.0;
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 13.0;

      l_lat_gps = lat_gps_previous;
      l_lon_gps = lon_gps_previous;

      lat_gps_previous = lat_gps_actual;
      lon_gps_previous = lon_gps_actual;

      gps_add_counter = 2;        // can 10ms cho mỗi lần đọc data giả, mà loop là 4ms -> counter =2.5 -> chọn 2
      new_gps_data_counter = 12;  // 100ms thì update gsp -> cần 13 dữ liệu để đọc mỗi 2 vòng loop: so lan lap gia
      lat_gps_add = 0;            //Reset the lat_gps_add variable.
      lon_gps_add = 0;
      new_gps_data_available = 1;
    }
  }

  //After 13 program loops 13 x 4ms ~ 100ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {  //If gps_add_counter is 0 and there are new GPS simulations needed.
    // Serial.println(gps_add_counter);
    new_gps_data_available = 2;  // data gia lap
    new_gps_data_counter--;      //Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 2;         //Set the gps_add_counter variable to 5 as a count down loop timer

    lat_gps_add += lat_gps_loop_add;    //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {        //If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;    //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;  //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }

    lon_gps_add += lon_gps_loop_add;    //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {        //If the absolute value of lat_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;    //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;  //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  }


  // --- TÍNH TOÁN PID GIỮ VỊ TRÍ ---
  if (new_gps_data_available) {
    new_gps_data_available = 0;
    if (flight_mode >= 3 && waypoint_set == 0) {
      Serial.println("Set waypoint");
      waypoint_set = 1;
      l_lat_waypoint = l_lat_gps;  // vi tri set point (target point)
      l_lon_waypoint = l_lon_gps;
    }

    if (flight_mode >= 3 && waypoint_set == 1) {  // pos hold mode
      // Tính toán sai số (Error)
      gps_lat_error = l_lat_waypoint - l_lat_gps;
      gps_lon_error = l_lon_waypoint - l_lon_gps;

      gps_lat_total_avarage -= gps_lat_rotating_mem[gps_rotating_mem_location];                  //Subtract the current memory position to make room for the new value.
      gps_lat_rotating_mem[gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;  //Calculate the new change between the actual pressure and the previous measurement.
      gps_lat_total_avarage += gps_lat_rotating_mem[gps_rotating_mem_location];                  //Add the new value to the long term avarage value.

      gps_lon_total_avarage -= gps_lon_rotating_mem[gps_rotating_mem_location];                  //Subtract the current memory position to make room for the new value.
      gps_lon_rotating_mem[gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;  //Calculate the new change between the actual pressure and the previous measurement.
      gps_lon_total_avarage += gps_lon_rotating_mem[gps_rotating_mem_location];                  //Add the new value to the long term avarage value.
      gps_rotating_mem_location++;                                                               //Increase the rotating memory location.
      if (gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                        //Start at 0 when the memory location 35 is reached.

      gps_lat_error_previous = gps_lat_error;  //Remember the error for the next loop.
      gps_lon_error_previous = gps_lon_error;

      // Tính toán Pitch/Roll điều chỉnh theo hướng Bắc
      gps_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;  // ap dung moving average filter cho Derivative
      gps_adjust_east = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      // Bù trừ theo bán cầu và xoay theo Yaw của máy bay (giữ nguyên logic source 293-300)
      if (!latitude_north) gps_adjust_north *= -1;
      if (!longitude_east) gps_adjust_east *= -1;

      gps_roll_pid_adjust = (-1.0) * (((float)gps_adjust_north * cos(yaw * 0.017453)) + ((float)gps_adjust_east * sin(yaw * 0.017453)));
      gps_pitch_pid_adjust = ((float)gps_adjust_east * cos(yaw * 0.017453)) - ((float)gps_adjust_north * sin(yaw * 0.017453));

      // Giới hạn đầu ra PID
      // gps_roll_pid_adjust = constrain(gps_roll_pid_adjust, -300, 300);
      // gps_pitch_pid_adjust = constrain(gps_pitch_pid_adjust, -300, 300);

      // Serial.print(gps_pitch_pid_adjust);
      // Serial.print(',');
      // Serial.println(gps_roll_pid_adjust);
    }
  }
  if (flight_mode < 3 && waypoint_set > 0) {  //If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_pid_adjust = 0;                  //Reset the gps_roll_pid_adjust variable to disable the correction.
    gps_pitch_pid_adjust = 0;                 //Reset the gps_pitch_pid_adjust variable to disable the correction.
    if (waypoint_set == 1) {                  //If the waypoints are stored
      gps_rotating_mem_location = 0;          //Set the gps_rotating_mem_location to zero so we can empty the
      waypoint_set = 2;                       //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[gps_rotating_mem_location] = 0;  //Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[gps_rotating_mem_location] = 0;  //Reset the current gps_lon_rotating_mem location.
    gps_rotating_mem_location++;                          //Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                   //Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
    }
  }
}

void gps_debug() {
  Serial.print(lat_gps_actual);
  Serial.print(',');
  Serial.println(lon_gps_actual);
  Serial.print(',');
  Serial.print(latitude_north);
  Serial.print(',');
  Serial.println(longitude_east);
  Serial.println(hdop);
  Serial.println(satellites);
}