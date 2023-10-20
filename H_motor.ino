void Motor(void){
  if(Switch>0){
   motor1 = Throttle_Control - Pitch_Out_rate + Roll_Out_rate + Yaw_Out_rate;//
   motor2 = Throttle_Control - Pitch_Out_rate - Roll_Out_rate - Yaw_Out_rate;//
   motor3 = Throttle_Control + Pitch_Out_rate + Roll_Out_rate - Yaw_Out_rate;//
   motor4 = Throttle_Control + Pitch_Out_rate - Roll_Out_rate + Yaw_Out_rate;//
   
       if(motor1 > 2000) motor1=2000;
  else if(motor1 < 1100) motor1=1100;
       if(motor2 > 2000) motor2=2000;
  else if(motor2 < 1100) motor2=1100; 
       if(motor3 > 2000) motor3=2000;
  else if(motor3 < 1100) motor3=1100; 
       if(motor4 > 2000) motor4=2000;
  else if(motor4 < 1100) motor4=1100;  
   
  }else {
   motor1 = 1000;//
   motor2 = 1000;//
   motor3 = 1000;//
   motor4 = 1000;//
  } 
   /* /////////////////////////////////////////////////////////
    *                             X  (roll)
    *                             ^
    *               motor 1       | motor 2
    *               clockwise     | counter clockwise 
    *               spin & blade  | spin & blade
    *                             |
    *               ______________|_____________ > Y(pitch)
    *                             |
    *               motor 3       | clockwise 
    *            counter clockwise| spin & blade
    *               spin & blade  | motor 4
    *                             |
    *                             |
    *                             |
    *                             
    *//////////////////////////////////////////////////////////
}
