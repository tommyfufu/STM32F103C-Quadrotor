#include <Wire.h>
#include<SoftWire.h>  
#include "Arduino.h"
#include <SPI.h>
#include <RF24-STM.h>

////This page is for variable declare////
////Gyro///////

///******Raw data******///
int16_t raw_gyro_x = 0, raw_gyro_y = 0, raw_gyro_z = 0;
int16_t raw_acc_x = 0, raw_acc_y = 0, raw_acc_z = 0;
int16_t raw_temp = 0;

///******Bias data******///
float   bias_gyro_x = -76.42;
float   bias_gyro_y = 13.69;
float   bias_gyro_z = 13.09;
float   bias_acc_x = -121.24;
float   bias_acc_y = -47.49;
float   bias_acc_z = 524.55;

///*******offset********///
//Bias Gyro = X:-80.64 Y:35.01 Z:16.02//v1
//Bias Accel = X:87.23 Y:-159.80 Z:541.97//v1

//Bias Gyro = X:-76.54 Y:1.62 Z:13.58//v2
//Bias Accel = X:106.76 Y:-54.21 Z:527.69//v2

//Bias Gyro = X:-76.42 Y:13.69 Z:13.09//   5.12
//Bias Accel = X:-121.24 Y:-47.49 Z:524.55//  5.12

///******Other******///
int32_t datacount;
float   gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
float    acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;

///******Gyro after calibration******///
float gx = 0, gy = 0, gz = 0;
float ax = 0, ay = 0, az = 0;
float Temp = 0;
float QAx, QAy, QAz;
///*******Gyro End line*********///

////Timer////
float dt = 0;
float oldt = 0;
int32_t loop_timer;
//////////LED/////////////
int8_t readledflag = 1;

/////*************************///
////Quartnion////

float q0 = 1;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float Ki, Kp;

float norm;
float vx, vy, vz;
float ex, ey, ez;
float exInt, eyInt, ezInt;


//////////RC///////////
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;

int16_t Roll_Control;
int16_t Pitch_Control;
int16_t Yaw_Control;
int16_t Throttle_Control;
int16_t boot;
int16_t Switch;

float Kgx;
float Kgy;

float divide = 5;///8 => 50 deg// 3= 133deg// 5=76deg//


/////////////////////////////////////////////////////////
/////////////////PID K gain////////////////////////////////////
///////////////////////////////////////////////////////
float roll_kp = 2.;
float roll_ki = 0.0;
float roll_kd = 0.0;
////////////////////////////////
float roll_rate_kp = 1.85; //still
float roll_rate_ki = 0.0085; //still
float roll_rate_kd = 2.85; //4.3
////////////////////////////
float yaw_kp = 1.0;
float yaw_ki = 0.0;
float yaw_kd = 0.0;
float yaw_rate_kp = 0.0;
float yaw_rate_ki = 0.0;
float yaw_rate_kd = 1.2; //no magnet
/////////////////////////////////////////////////////////
/////////////////PID K gain////////////////////////////////////
///////////////////////////////////////////////////////
//////////////////////////////////////////////
float roll_rate_MAX = 400; // 增益上限+- 400 pwm
float roll_deg_MAX = 160; // 增益上限+-50 degree
float roll_rate_MAX_I = 160;

float roll_deg_expect = 0;
float roll_deg_current = 0;
float roll_deg_error = 0;
float roll_last_deg_error = 0;
float roll_KI_deg_out = 0;
float roll_KP_deg_out = 0;
float roll_KD_deg_out = 0;
float Roll_Out_deg = 0;
float roll_rate_expect = 0;
float roll_rate_current = 0;
float roll_rate_error = 0;
float roll_last_rate_error = 0;
float roll_KI_rate_out = 0;
float roll_KP_rate_out = 0;
float roll_KD_rate_out = 0;
float Roll_Out_rate = 0;

//////////PID PITCH gain///////////
float pitch_kp = roll_kp;
float pitch_ki = roll_ki;
float pitch_kd = roll_kd;


float pitch_rate_kp = roll_rate_kp;
float pitch_rate_ki = roll_rate_ki;
float pitch_rate_kd = roll_rate_kd;
float pitch_rate_MAX_I = roll_rate_MAX_I;

//////////////////////////////////////////////////
float pitch_deg_MAX = 160; // 增益上限+-160 degree
float pitch_rate_MAX = 400; // 增益上限+- 400 pwm
float pitch_deg_expect = 0;
float pitch_deg_current = 0;
float pitch_deg_error = 0;
float pitch_last_deg_error = 0;
float pitch_KI_deg_out = 0;
float pitch_KP_deg_out = 0;
float pitch_KD_deg_out = 0;
float Pitch_Out_deg = 0;
float pitch_rate_expect = 0;
float pitch_rate_current = 0;
float pitch_rate_error = 0;
float pitch_last_rate_error = 0;
float pitch_KI_rate_out = 0;
float pitch_KP_rate_out = 0;
float pitch_KD_rate_out = 0;
float Pitch_Out_rate = 0;
//////////PID YAW gain///////////
float yaw_deg_MAX = 160; // 增益上限+-160 degree
float yaw_rate_MAX = 400; // 增益上限+- 400 pwm
float yaw_deg_expect = 0;
float yaw_deg_current = 0;
float yaw_deg_error = 0;
float yaw_last_deg_error = 0;
float yaw_KI_deg_out = 0;
float yaw_KP_deg_out = 0;
float yaw_KD_deg_out = 0;
float Yaw_Out_deg = 0;
float yaw_rate_expect = 0;
float yaw_rate_current = 0;
float yaw_rate_error = 0;
float yaw_last_rate_error = 0;
float yaw_KI_rate_out = 0;
float yaw_KP_rate_out = 0;
float yaw_KD_rate_out = 0;
float Yaw_Out_rate = 0;
///////////motor////幫你從int
uint16_t motor1 = 0;
uint16_t motor2 = 0;
uint16_t motor3 = 0;
uint16_t motor4 = 0;
/***********NRF24L01+角位及取樣頻率參數**********/
const uint16_t CE = PA11;
const uint16_t CSN = PA12;
const uint16_t nrf_irq = PB4;
RF24 radio(CE, CSN);
uint8_t send_count = 0; //NRF取樣頻率
/******傳輸封包******/
struct Data_Package {
  float QAx;
  float QAy;
  float QAz;
  uint16_t motor1;
  uint16_t motor2;
  uint16_t motor3;
  uint16_t motor4;
};
Data_Package nrf_package;//傳3個float 4個u8 共20bytes


/*************************************/

void setup() {
  initial();///seting evering in this funtion
}

void loop() {
  flightcontrol();///flight control program
}
