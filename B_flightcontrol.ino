void flightcontrol() {
  loop_timer = micros();                                        //Set the timer for the first loop.
  indecator_boot_alert(Switch);
  control_Signal();
  control_Gyro();
  set_unit();
  calculate_quaternion();
  QAz = QAz * 0.96 + gz * 0.04;
  PID_outter();
  PID_inner();
  Motor();

  ///////////turn on?off motor ///////////////
  TIMER4_BASE->CCR1 = motor1; //PB6   Blue
  TIMER4_BASE->CCR2 = motor2; //PB7   Green
  TIMER4_BASE->CCR3 = motor3; //PB8   Yellow
  TIMER4_BASE->CCR4 = motor4; //PB9   Orange
  TIMER4_BASE->CNT = 5000; //This will reset timer 4 and the ESC pulses are directly created
  /*Loop每跑5(send_count)次就傳送(Sending_Data())，
    目前dronev0試設5沒啥問題，以不影響飛行為前提去設*/
  send_count++;
  if (send_count == 5) {
    Sending_Data(QAx, QAy, QAz, motor1, motor2, motor3, motor4);
    send_count = 0;
  }

  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();

}
