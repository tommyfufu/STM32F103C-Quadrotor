void indecator_boot_alert(uint8_t flag){
  if(flag==1){
    readledflag++;
    if(readledflag==10)readledflag*=-1;   
    if(readledflag>0)indecator_Bee(0);
    else indecator_Bee(1);  
  } else indecator_Bee(0);
}

void indicate_biasBee(void){
  indecator_Bee(1);
  delay(500);
  indecator_Bee(0);
  delay(40);
  indecator_Bee(1);
  delay(40);
  indecator_Bee(0);
  delay(40);
  indecator_Bee(1);
  delay(40);
  indecator_Bee(0);
  delay(40);
  indecator_Bee(1);
  delay(40);
  indecator_Bee(0);
}

void indecator_onChipLED(int8_t set) {
  if(set == 1)  digitalWrite(PC13,HIGH);
  if(set == 0)  digitalWrite(PC13,LOW); 
}

void indecator_Bee(int8_t set) {
  if(set == 1)  digitalWrite(PB12,HIGH);
  if(set == 0)  digitalWrite(PB12,LOW);
}
