char str[20];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
}
unsigned long tim = 0;
void loop() {
  //Serial.println("Goodbye World");
  if(millis() - tim >= 5000){
  Serial1.write(1);
  tim = millis();}
  }

void serialEvent1(){
        Serial1.readBytes(str, 10);
      Serial.println(str);
}
