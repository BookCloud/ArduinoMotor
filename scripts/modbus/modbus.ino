#include <ModbusMaster.h>               //Library for using ModbusMaster

#define MAX485_DE      3                 //put for RSE pin
#define MAX485_RE_NEG  2
#define MAX485_DE2      7                //put for RSE pin
#define MAX485_RE_NEG2  6
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

const int encoderLB = 18;
const int encoderLA = 19;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;

boolean newData = false;
String spd;
int pos;
int in =0;
ModbusMaster node;                    //object node for class ModbusMaster
ModbusMaster node2; 
void preTransmission()            //Function for setting stste of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);             
  digitalWrite(MAX485_DE, 1);
  digitalWrite(MAX485_RE_NEG2, 1);             
  digitalWrite(MAX485_DE2, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE_NEG2, OUTPUT);
  pinMode(MAX485_DE2, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(9600);             //Default Baud Rate of motor as 115200

  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoder, CHANGE);
 

    node.begin(2, Serial3);            //Slave ID as 4, serialport 3
  node2.begin(4, Serial1);
  
  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);
  node2.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  node2.postTransmission(postTransmission);
  
  //node.writeSingleRegister(0x200F,1);
  //node2.writeSingleRegister(0x200F,1);
  node.writeSingleRegister(0x2032,3); // operating mode, velocity mode
node.writeSingleRegister(0x2031,8); // control, enable motor
node2.writeSingleRegister(0x2032,3); // operating mode, velocity mode
node2.writeSingleRegister(0x2031,8); // control, enable motor
  
  
}


void loop()
{

//Write to the the motor speed in RPM
node.writeSingleRegister(0x203A,100); //target speed, rpm
node2.writeSingleRegister(0x203A,100); //target speed, rpm




/*Reads the encoder values using 2 registers and each [getResponseBuffer] reads 16bit value but encoder feedback is 32bit 
  thus requiring 2 register to read encoder feedback*/
uint16_t result;
result = node.readHoldingRegisters(0x202A,2);
Serial.println(node.getResponseBuffer(0)); // After every 65536 count this register counts +1
Serial.println(node.getResponseBuffer(1)); // Gives encoder from 0 till 65536



//node.writeSingleRegister(0x203A,100); //target speed, rpm
//node2.writeSingleRegister(0x203A,100); //target speed, rpm

    //recvWithEndMarker();
    //showNewData();

   //uint16_t result;
   //result = node.readHoldingRegisters(0x200D,1);
   //Serial.println(result);
  // Serial.println(encoderValue);//node.getResponseBuffer(0));
   //Serial.println(node.getResponseBuffer(0));
  

   
 //  delay(500);
   
    


  //delay(100);
  //node.writeSingleRegister(0x2031,7); //control, stop
  //delay(500);//Writes value to 0x40000 holding register
}



void updateEncoder(){
int MSB = digitalRead(encoderLB); //MSB = most significant bit
int LSB = digitalRead(encoderLA); //LSB = least significant bit
int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {encoderValue++;}
if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {encoderValue--;}
//17000 is the current 1 revolution encoder feedback with ch 1 as A-and ch2 as B+
lastEncoded = encoded; //store this value for next time
}     
