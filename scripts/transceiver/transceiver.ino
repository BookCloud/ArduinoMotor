char c[10];
String a;
unsigned long x = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

unsigned long tim = 0;
unsigned long tim2 = 0;


void loop() {
  //Serial.println("Hello World");

  if(millis() -tim2 >=100){
    x++;
    tim2 = millis();
  }}


void serialEvent(){
  a = String(x);
  a.toCharArray(c, 10);
  Serial.write(c, 10);
  Serial.read();
}
