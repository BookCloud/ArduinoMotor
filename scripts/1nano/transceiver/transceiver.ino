char c[99];
String a;
char c2[10];
String a2;
unsigned long x = 0;
unsigned long y = 0;
int trans = -6969;
int16_t trans2 = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(1000000);
 // Serial.println("Hello World");

}

unsigned long tim = 0;
unsigned long tim2 = 0;
int8_t xb = B11110110;  // the 0b prefix indicates a binary constant


void loop() {
  //Serial.println("Hello World");
  if(millis() - tim >= 222){
            //trans2 ++;
           tim = millis();
  }
  
  if(millis() -tim2 >=15){
   

    Serial1.write(0x3C);

    int8_t LSB1 = trans;
    int8_t MSB1 = trans >> 8;
    Serial1.write(MSB1);
    Serial1.write(LSB1);

    int8_t LSB2 = trans2;
    int8_t MSB2 = trans2 >> 8;
    Serial1.write(MSB2);
    Serial1.write(LSB2);


    //Serial1.write(B11110110);
    //Serial1.write(B11110101);

    //Serial1.write(trans2/256);
    //Serial1.write(trans2%256);
    
    //Serial1.write(160);
    //Serial1.write(B11110000);

    //Serial1.write(0x69);
    
    Serial1.write(0x3E);
    //sen();
      tim2 = millis();
}
  
  //event();

  
  }

void sen(){
  Serial1.write("12345");     // send high byte
  //Serial.println("s");
  //Serial1.write(359 && 0xFF);  // send low byte
}


void event(){
  if(Serial1.available() > 0){
   Serial.println(Serial1.read());
   Serial1.write("111111,2222222");
  }
}

void serialEvent(){
  a = String(x);
  a2 = String(y);
  a.concat(",");
  a.concat(a2);

  //strcpy(c,y);
  //puts(c);
  a.toCharArray(c, 99);
  Serial1.write("a");
  Serial.println("hi");
  Serial1.read();
}
