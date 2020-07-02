#include <ModbusMaster.h>               //Library for using ModbusMaster

#define MAX485_DE      3
#define MAX485_RE_NEG  2
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;
String spd;
int pos;
int in =0;
ModbusMaster node;                    //object node for class ModbusMaster

void preTransmission()            //Function for setting stste of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);             
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  Serial.begin(9600);
  
  Serial3.begin(9600);             //Baud Rate as 115200

  node.begin(4, Serial3);            //Slave ID as 4, serialport 3
  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);

  
  node.writeSingleRegister(0x2032,3); // operating mode, velocity mode
  node.writeSingleRegister(0x2031,8); // control, enable motor

}


void loop()
{
  
    recvWithEndMarker();
    showNewData();
    node.writeSingleRegister(0x203A,pos); //target speed, rpm


  //delay(100);
  //node.writeSingleRegister(0x2031,7); //control, stop
  //delay(500);//Writes value to 0x40000 holding register
}



void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
    
}


void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        spd = receivedChars;
        pos = spd.toInt();
        newData = false;
    }
}
