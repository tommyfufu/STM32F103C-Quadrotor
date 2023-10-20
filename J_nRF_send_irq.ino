void Sending_Data(float QAx,float QAy,float QAz,uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4){

    nrf_package.QAx=QAx;
    nrf_package.QAy=QAy;
    nrf_package.QAz=QAz;
    nrf_package.motor1=motor1;
    nrf_package.motor2=motor2;
    nrf_package.motor3=motor3;
    nrf_package.motor4=motor4;
    radio.stopListening();
    radio.write(&nrf_package, sizeof(Data_Package));
    radio.startListening();
}
void check_radio(void)
{  
  // What happened?
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);
//  if(tx)Serial.println("TxOK");
//  if(fail)Serial.println("Txfail");
  if(rx)Emergent_Control();  
}
void Emergent_Control(void){
    int con_by_mon = 0;                 //Saving the incoming data
    radio.read(&con_by_mon, sizeof(int));    //Reading the data
    delay(1);
//    Serial.println(con_by_mon);
    if(con_by_mon!=0){
      //這邊幫我寫一個關掉drone的程式碼
      Serial.println("Turn off the Drone");
    }
    else
      Serial.println("Still Flying");
}
