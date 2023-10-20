void calculate_GyroBias(void) {
  for(datacount=0;datacount<2000 ;datacount++)
  {
    ReadGyroData();
    if(datacount%2 == 0){
      Serial.print(datacount);
      Serial.println("Calculating Bias ......");
    }
    acc_x_sum += raw_acc_x;
    acc_y_sum += raw_acc_y;
    acc_z_sum += raw_acc_z;
    gyro_x_sum += raw_gyro_x;
    gyro_y_sum += raw_gyro_y;
    gyro_z_sum += raw_gyro_z;
  }
  
  /////////////////////////////////
  
  bias_acc_x = acc_x_sum / 2000 ;
  bias_acc_y = acc_y_sum / 2000 ;
  bias_acc_z = (acc_z_sum - 8192000)/2000;
  bias_gyro_x = gyro_x_sum/ 2000 ;
  bias_gyro_y = gyro_y_sum/ 2000 ;
  bias_gyro_z = gyro_z_sum/ 2000 ;
  
  ///////////////////////////////
}

void calculate_quaternion(void)
{
  Ki=0.001;
  Kp=1.6;
  float norm;
  float vx, vy, vz;
  float ex, ey, ez; 

 
  //加计值归一化处理normalise the measurements
  norm = sqrtf(ax*ax + ay*ay + az*az);      
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;      

  //建立飞控坐标系estimated direction of gravity
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  //坐标系和重力叉积运算error is sum of cross product between reference direction of field and direction measured by sensor
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);

  //比例运算integral error scaled integral gain
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;

  //陀螺仪积分融合 adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
  //计算四元数 integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*dt*0.5;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*dt*0.5;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*dt*0.5;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*dt*0.5;  

  //四元数归一化处理normalise quaternion
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  //计算欧拉角
  QAx = atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1-2*q2*q2 + 1)*57.29;//正负180度
  QAy = asinf(-2*q1*q3 + 2*q0*q2)*57.29;//正负90度
  QAz = atan2f(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.29;  //乘以57.30是为了将弧度转化为角度
}
