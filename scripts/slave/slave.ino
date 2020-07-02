

#include<ModbusRtu.h>       //Library for using Modbus in Arduino
                      //Initilize servo object for class Servo
Modbus bus;                          //Define Object bus for class modbus 
uint16_t modbus_array[] = {0,0,0};    //Array initilized with three 0 values
                      
void setup()
{
  bus = Modbus(1,1,4);            //Modbus slave ID as 1 and 1 connected via RS-485 and 4 connected to DE & RE pin of RS-485 Module 
  bus.begin(9600);                //Modbus slave baudrate at 9600
}

void loop()

{
  Serial.println(bus.read();)
}
