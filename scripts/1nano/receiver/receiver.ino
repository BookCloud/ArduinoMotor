
// Example 6 - Receiving binary data

const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;

void setup() {
    Serial.begin(2000000);
    Serial3.begin(1000000);
    Serial1.begin(9600);
    Serial2.begin(9600);
    Serial.println("<Arduino is ready>");

}

void loop() {
    Serial1.write("heool");
    Serial2.write("world");
    recvBytesWithStartEndMarkers();
    showNewData();
}

void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rb;
   

    while (Serial3.available() > 0 && newData == false) {
        rb = Serial3.read();

        if (recvInProgress == true) {
            if (rb != endMarker) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("This... ");
        for (byte n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes[n], DEC );
            Serial.print(' ');
        }
        pars();     
        newData = false;
    }
}

void pars(){
  Serial.print("Parsed: ");

   int re = pars(receivedBytes[0], receivedBytes[1]);
  int le = pars(receivedBytes[2], receivedBytes[3]);
  //Serial.println();
  Serial.print(re,DEC);
  Serial.print(" ");
  Serial.print(le,DEC);

    Serial.println();



}


int pars(uint8_t MSB, uint8_t LSB){
  

  int rec = (MSB << 8) | LSB;
  
  //Serial.print(MSB, BIN);
  //Serial.print(' ');
  //Serial.print(LSB, BIN);
  //Serial.print(' ');
  //Serial.print(rec);
  return rec;
}
