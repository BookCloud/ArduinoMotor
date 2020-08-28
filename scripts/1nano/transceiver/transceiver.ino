unsigned long x = 0;
unsigned long y = 0;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

unsigned long tim = 0;
unsigned long tim2 = 0;
long trans = 0;
long trans2 = 0;

void loop() {
  //interval to increment data to be transfered
  if(micros() - tim >= 100){
           trans ++;
           trans2 --;
           tim = micros();
  }

 // transfer data
 event();
}

  
void event(){
  //interval to send data
  if(millis() -tim2 >= 30){
        
    Serial1.write(0x3C); //start marker "<" (ascii to hex)


    //split 32bit integer to four 8bit
    int8_t LSB = trans;
    int8_t MSB = trans >> 8;
    int8_t LSB2 = trans >> 16;
    int8_t MSB2 = trans >> 24;

    int8_t LSB31 = trans2;
    int8_t MSB31 = trans2 >> 8;
    int8_t LSB231 = trans2 >> 16;
    int8_t MSB231 = trans2 >> 24;

/*
    Serial.print(LSB, BIN);
    Serial.print(" ");
    Serial.print(MSB, BIN);
    Serial.print(" ");
    Serial.print(LSB2, BIN);
    Serial.print(" ");
    Serial.println(MSB2, BIN);
    */
    
    //send data
    Serial1.write(LSB);
    Serial1.write(MSB);
    Serial1.write(LSB2);
    Serial1.write(MSB2);

    /*int8_t LSB2 = trans2;
    int8_t MSB2 = trans2 >> 8;
    Serial1.write(MSB2);
    Serial1.write(LSB2);*/


    Serial1.write(LSB31);
    Serial1.write(MSB31);
    Serial1.write(LSB231);
    Serial1.write(MSB231);
    
    Serial1.write(0x3E); //end marker ">"
    tim2 = millis();
}
}
