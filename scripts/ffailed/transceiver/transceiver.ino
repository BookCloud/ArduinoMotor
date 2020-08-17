char c[99];
String a;
char c2[10];
String a2;
unsigned long x = 0;
unsigned long y = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

unsigned long tim = 0;
unsigned long tim2 = 0;


void loop() {
  //Serial.println("Hello World");

  if(millis() -tim2 >=10){
   
    //x++;
    //y--;
    tim2 = millis();
    sen();
    
  }
  

  
  }


void sen(){
  a = String(x);
  a2 = String(y);
  a.concat(",");
  a.concat(a2);

  //strcpy(c,y);
  //puts(c);
  a.toCharArray(c, 99);
  Serial.write("160000,160000");
  Serial.read();
}
