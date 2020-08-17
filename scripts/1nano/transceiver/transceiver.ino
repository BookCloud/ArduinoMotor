char c[99];
String a;
char c2[10];
String a2;
unsigned long x = 0;
unsigned long y = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
 // Serial.println("Hello World");

}

unsigned long tim = 0;
unsigned long tim2 = 0;


void loop() {
  //Serial.println("Hello World");

  if(millis() -tim2 >=1000){
   
    //x++;
    //y--;
   // Serial1.write("160000,160000");
    sen();
    
      tim2 = millis();
}
  
  //event();

  
  }

void sen(){
  Serial1.write(359 >> 8);     // send high byte
Serial1.write(359 && 0xFF);  // send low byte
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
