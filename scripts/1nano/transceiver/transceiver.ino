char c[99];
String a;
char c2[10];
String a2;
unsigned long x = 0;
unsigned long y = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial1.begin(1000000);
  Serial.println("Hello World");
   //test();
}

unsigned long tim = 0;
unsigned long tim2 = 0;
int8_t xb = B11110110;  // the 0b prefix indicates a binary constant

void loop() {
  //Serial.println("Hello World");
  if(micros() - tim >= 100){
           //trans ++;
           tim = micros();
  }
 event();
  }

  
void test(){
  
  int32_t x = -1600000;
  int32_t a4= x >> 24;
  int32_t a3 = x >> 16;
  int32_t a2 = x >> 8;
  int32_t a1 = x;

  uint32_t MSB2= a4;
  uint32_t LSB2 = a3;
  uint32_t MSB = a2;
  uint32_t LSB = a1;
  Serial.println(x);
  Serial.println(LSB, BIN);
  Serial.println(MSB, BIN);
  Serial.println(LSB2, BIN);
  Serial.println(MSB2, BIN);
  Serial.println();
  Serial.print(MSB2, BIN);
  Serial.print(LSB2, BIN);
  Serial.print(MSB, BIN);
  Serial.print(LSB, BIN);
  Serial.println();
  Serial.println(x, BIN);
  Serial.println();

  int32_t p = (MSB<<8) | LSB;
  int32_t q = (LSB2<<16) | p;
  int32_t o = (MSB2<<24) | q;
  int32_t f = o;
    /* 11111111 11100111 10010110 00000000
   * 11111111 11111111 10010110 00000000                
              11100111 10010110   
   
   
  
   
   */
  Serial.println(LSB, BIN);
  //Serial.println(x, BIN);
  Serial.println(p, BIN);
  //Serial.println(x, BIN);
  Serial.println(LSB2<<16, BIN);
  Serial.println(o, BIN);
    Serial.println();
    Serial.println(f, DEC);
    Serial.println(f, BIN);

}


int32_t trans = -12345678;
int32_t trans2 = -91620000;

void event(){
  if(millis() -tim2 >= 500){
        
    Serial1.write(0x3C);

    
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
    Serial1.write(LSB);
    Serial1.write(MSB);
    Serial1.write(LSB2);
    Serial1.write(MSB2);

    /*int8_t LSB2 = trans2;
    int8_t MSB2 = trans2 >> 8;
    Serial1.write(MSB2);
    Serial1.write(LSB2);*/


    //Serial1.write(B11110110);
    //Serial1.write(B11110101);

    //Serial1.write(trans2/256);
    //Serial1.write(trans2%256);
    
    //Serial1.write(160);
    //Serial1.write(B11110000);

    //Serial1.write(0x69);

    Serial1.write(LSB31);
    Serial1.write(MSB31);
    Serial1.write(LSB231);
    Serial1.write(MSB231);
    
    Serial1.write(0x3E);
    tim2 = millis();
}
}
