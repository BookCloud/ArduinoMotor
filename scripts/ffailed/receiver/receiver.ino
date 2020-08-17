
char *str[10] ;
char msg[32];
char *p = NULL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
}
unsigned long tim = 0;
void loop() {
  //Serial.println("Goodbye World");

  
  if(millis() - tim >= 1000){
  //Serial1.write(1);
  Serial3.readBytes(msg,20);
  Serial.println(msg);
  tim = millis();}

  
  }

void rec(){
  
  Serial3.readBytes(msg, 20);

    byte index = 0;
    p = strtok(msg, ",");  // takes a list of delimiters
    while(p != NULL)
    {
        str[index] = p;
        index++;
        p = strtok(NULL, ",");  // takes a list of delimiters
    }
    //Serial.println(index);
// print the tokens
    Serial.print("Left encoder is ");
    Serial.println(str[0]);
        Serial.print("right encoder is ");

    Serial.println(str[1]);

}
  
  //while ((str = strtok_r(p, ",", &p)) != "\n")
 //   {
//     Serial.println(str);
///     p = NULL;
//    }
 //   }
