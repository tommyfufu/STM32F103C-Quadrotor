
void PID_outter(void){
  PID_get_Deg();///read current degree
  PID_Signal_Control();////read current control signal
  PID_deg();///calculate the except degree for inner loop
}

void PID_get_Deg(void){///using quaternion for reference
  roll_deg_current  = QAx;
  pitch_deg_current = QAy;
  yaw_deg_current   = QAz;
}

void PID_Signal_Control(void){ 
  ///the range for control signal is 1100 to 1900///
  // "signal/divide" is the way I mapping data to the except degree//
  
       if (Roll_Control > 1520) roll_deg_expect = (Roll_Control - 1520) / divide; 
  else if (Roll_Control < 1480) roll_deg_expect = (Roll_Control - 1480) / divide;
  else roll_deg_expect = 0;

       if (Pitch_Control > 1520) pitch_deg_expect = (Pitch_Control - 1520) / divide;
  else if (Pitch_Control < 1480) pitch_deg_expect = (Pitch_Control - 1480) / divide;
  else pitch_deg_expect = 0;

  
       if (Yaw_Control > 1520) yaw_deg_expect = (Yaw_Control - 1520) / divide;
  else if (Yaw_Control < 1480) yaw_deg_expect = (Yaw_Control - 1480) / divide;
  else yaw_deg_expect = 0;
 
}

void PID_deg(void){
  ///Calculate except degree///
  roll_deg_error  = roll_deg_expect  - roll_deg_current;
  pitch_deg_error = pitch_deg_expect - pitch_deg_current;
  yaw_deg_error   = yaw_deg_expect   - yaw_deg_current;
  
  roll_KP_deg_out  = roll_kp  * roll_deg_error;
  pitch_KP_deg_out = pitch_kp * pitch_deg_error;
  yaw_KP_deg_out   = yaw_kp   * yaw_deg_error;
  
  roll_KD_deg_out  = roll_kd  * (roll_deg_error   - roll_last_deg_error);
  pitch_KD_deg_out = pitch_kd * (pitch_deg_error  - pitch_last_deg_error);
  yaw_KD_deg_out   = yaw_kd   * (yaw_deg_error    - yaw_last_deg_error);

  
  /////////////////KI /////////////
  roll_KI_deg_out  += roll_ki  * roll_deg_error; 
  pitch_KI_deg_out += pitch_ki * pitch_deg_error; 
  yaw_KI_deg_out   += yaw_ki   * yaw_deg_error; 
/////////////////KI /////////////


  ///limited the KI
       if(roll_KI_deg_out>roll_deg_MAX)       roll_KI_deg_out = roll_deg_MAX;
  else if(roll_KI_deg_out < roll_deg_MAX*-1)  roll_KI_deg_out = roll_deg_MAX*-1;

       if(pitch_KI_deg_out>pitch_deg_MAX)      pitch_KI_deg_out = pitch_deg_MAX;
  else if(pitch_KI_deg_out < pitch_deg_MAX*-1) pitch_KI_deg_out = pitch_deg_MAX*-1;

       if(yaw_KI_deg_out>yaw_deg_MAX)      yaw_KI_deg_out = yaw_deg_MAX;
  else if(yaw_KI_deg_out < yaw_deg_MAX*-1) yaw_KI_deg_out = yaw_deg_MAX*-1;
  
  Roll_Out_deg  = roll_KP_deg_out   + roll_KI_deg_out   + roll_KD_deg_out;//
  Pitch_Out_deg = pitch_KP_deg_out  + pitch_KI_deg_out  + pitch_KD_deg_out;//
  Yaw_Out_deg   = yaw_KP_deg_out    + yaw_KI_deg_out    + yaw_KD_deg_out;//


  
  
       if(Roll_Out_deg>roll_deg_MAX)     Roll_Out_deg = roll_deg_MAX;
  else if(Roll_Out_deg<roll_deg_MAX*-1)  Roll_Out_deg =  roll_deg_MAX*-1;

       if(Pitch_Out_deg>pitch_deg_MAX)    Pitch_Out_deg = pitch_deg_MAX;
  else if(Pitch_Out_deg<pitch_deg_MAX*-1) Pitch_Out_deg = pitch_deg_MAX*-1;
  
       if(Yaw_Out_deg>yaw_deg_MAX)    Yaw_Out_deg = yaw_deg_MAX;
  else if(Yaw_Out_deg<yaw_deg_MAX*-1) Yaw_Out_deg = yaw_deg_MAX*-1;
  
  roll_last_deg_error   = roll_deg_error;
  pitch_last_deg_error  = pitch_deg_error;
  yaw_last_deg_error    = yaw_deg_error;
}




void PID_inner(void){
  PID_get_Rate();///讀取當前角度
  PID_Control_Rate();////讀取當前控制訊號
  PID_rate();////計算外環期望角度
}

void PID_get_Rate(void){
  roll_rate_current   = gx  * 57.29; //get the current angle rate
  pitch_rate_current  = gy  * 57.29; //get the current angle rate
  yaw_rate_current    = gz  * 57.29; //get the current angle rate
}

void PID_Control_Rate(void){ 
    roll_rate_expect  = Roll_Out_deg;
    pitch_rate_expect = Pitch_Out_deg;
    yaw_rate_expect   = Yaw_Out_deg;
}

void PID_rate(void){
  //calculate the except degree rate
  roll_rate_error   = roll_rate_expect  - roll_rate_current;
  pitch_rate_error  = pitch_rate_expect - pitch_rate_current;
  yaw_rate_error    = yaw_rate_expect   - yaw_rate_current;

  
  
  roll_KP_rate_out  = roll_rate_kp  * roll_rate_error;
  pitch_KP_rate_out = pitch_rate_kp * pitch_rate_error;
  yaw_KP_rate_out   = yaw_rate_kp   * yaw_rate_error;
  
  roll_KD_rate_out  = roll_rate_kd  * (roll_rate_error - roll_last_rate_error);
  pitch_KD_rate_out = pitch_rate_kd * (pitch_rate_error - pitch_last_rate_error);
  yaw_KD_rate_out   = yaw_rate_kd   * (yaw_rate_error - yaw_last_rate_error);


  roll_KI_rate_out  += roll_rate_ki   * roll_rate_error; 
  pitch_KI_rate_out += pitch_rate_ki  * pitch_rate_error; 
  yaw_KI_rate_out   += yaw_rate_ki    * yaw_rate_error;

  if(Throttle_Control < 1050 || Switch == 0 ){
      roll_KI_rate_out = 0;
      pitch_KI_rate_out = 0;
      yaw_KI_rate_out = 0;
    }

       if(roll_KI_rate_out>roll_rate_MAX_I)       roll_KI_rate_out  = roll_rate_MAX_I;
  else if(roll_KI_rate_out < roll_rate_MAX_I*-1)  roll_KI_rate_out  = roll_rate_MAX_I*-1;

       if(pitch_KI_rate_out>pitch_rate_MAX_I)       pitch_KI_rate_out = pitch_rate_MAX_I;
  else if(pitch_KI_rate_out < pitch_rate_MAX_I*-1)  pitch_KI_rate_out = pitch_rate_MAX_I*-1;

       if(yaw_KI_rate_out>yaw_rate_MAX)       yaw_KI_rate_out = yaw_rate_MAX;
  else if(yaw_KI_rate_out < yaw_rate_MAX*-1)  yaw_KI_rate_out = yaw_rate_MAX*-1;
  
  Roll_Out_rate   = roll_KP_rate_out  + roll_KD_rate_out  + roll_KI_rate_out;
  Pitch_Out_rate  = pitch_KP_rate_out + pitch_KD_rate_out + pitch_KI_rate_out;
  Yaw_Out_rate    = yaw_KP_rate_out   + yaw_KD_rate_out   + yaw_KI_rate_out;

       if(Roll_Out_rate>roll_rate_MAX)    Roll_Out_rate = roll_rate_MAX;
  else if(Roll_Out_rate<roll_rate_MAX*-1) Roll_Out_rate = roll_rate_MAX*-1;
  
       if(Pitch_Out_rate>pitch_rate_MAX)    Pitch_Out_rate  = pitch_rate_MAX;
  else if(Pitch_Out_rate<pitch_rate_MAX*-1) Pitch_Out_rate  = pitch_rate_MAX*-1;
 
       if(Yaw_Out_rate>yaw_rate_MAX)    Yaw_Out_rate  = yaw_rate_MAX;
  else if(Yaw_Out_rate<yaw_rate_MAX*-1) Yaw_Out_rate  = yaw_rate_MAX*-1;

  roll_last_rate_error=roll_rate_error;
  pitch_last_rate_error=pitch_rate_error;
  yaw_last_rate_error=yaw_rate_error;
}
