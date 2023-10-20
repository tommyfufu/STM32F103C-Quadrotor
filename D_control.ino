
void control_Signal(void){
  
  Roll_Control      = channel_1;
  Pitch_Control     = channel_2;
  Yaw_Control       = channel_4;
  Throttle_Control  = channel_3;
  boot              = channel_6;

  if(Switch >0 ){
         if(Roll_Control > 1900) Roll_Control = 1900;
    else if(Roll_Control < 1100) Roll_Control = 1100;
     
         if(Pitch_Control > 1900) Pitch_Control = 1900;
    else if(Pitch_Control < 1100) Pitch_Control = 1100;
  
         if(Yaw_Control > 1900) Yaw_Control = 1900;
    else if(Yaw_Control < 1100) Yaw_Control = 1100;
  
    if(Throttle_Control>1900) Throttle_Control = 1900;
    
  }else if (!Switch){
    Roll_Control=1500;
    Pitch_Control=1500;
    Yaw_Control=1500;
    Throttle_Control=1000;   
  }
  //////////throttle & switch////////
       if ( boot > 1500) Switch = 1;
  else if ( boot < 1500) Switch = 0;
  
  boot = 0;
}


void control_Gyro(void) {
  ReadGyroData();
  ax=raw_acc_x - bias_acc_x;
  ay=raw_acc_y - bias_acc_y;
  az=raw_acc_z - bias_acc_z;
  Temp=(double)(raw_temp/340)+36.53;
  gx =raw_gyro_x - bias_gyro_x;
  gy =raw_gyro_y - bias_gyro_y;
  gz =raw_gyro_z - bias_gyro_z;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadGyroData(void) {
  dt=(millis()-oldt)/1000;
  oldt=millis();  
  Wire.beginTransmission(0x68);                       //Start communication with the gyro.
  Wire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                     //End the transmission.
  Wire.requestFrom(0x68, 14);                         //Request 14 bytes from the MPU 6050.
  raw_acc_x = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_x variable.
  raw_acc_y = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_y variable.
  raw_acc_z = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_z variable.
  raw_temp = Wire.read() << 8 | Wire.read();              //Add the low and high byte to the temperature variable.
  raw_gyro_x = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
  raw_gyro_y = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data.
  raw_gyro_z = Wire.read() << 8 | Wire.read();                 //Read high and low part of the angular data.
}
