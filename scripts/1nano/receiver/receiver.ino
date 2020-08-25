
// Example 6 - Receiving binary data

const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;

void setup() {
    Serial.begin(1000000);
    Serial2.begin(1000000);
    Serial.println("<Arduino is ready>");
    int y = (2<<8);
    int x = (y << 8);
    //Serial.print((y << 8), BIN);
}

void loop() {
    clearbuffer();
    recvBytesWithStartEndMarkers();
    showNewData();
}

void clearbuffer(){
  while(Serial2.available() >0){
    Serial2.read();
  }
}
void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rb;

    while(Serial2.available() <= 0){
      int wait = 0;
    }
    while (Serial2.available() > 0 && newData == false) {
        rb = Serial2.read();

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
        //Serial.print("This... ");
        for (byte n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes[n], BIN );
            Serial.print(' ');
        }
        pars();     
        newData = false;
    }
}

void pars(){

  //int re = pars(receivedBytes[0], receivedBytes[1]);
  //int le = pars(receivedBytes[2], receivedBytes[3]);
  //Serial.println();
  //Serial.print(re,DEC);
  //Serial.print(" ");
  //Serial.print(le,DEC);

    int32_t p = test(receivedBytes[0], receivedBytes[1], receivedBytes[2], receivedBytes[3] );
    int32_t q = test(receivedBytes[4], receivedBytes[5], receivedBytes[6], receivedBytes[7] );

    Serial.println();
    Serial.print("Parsed: ");
    Serial.print(p);
    Serial.print(" ");
    Serial.println(q);




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

int32_t test(uint32_t LSB, uint32_t MSB, uint32_t LSB2, uint32_t MSB2){
  int32_t two = (MSB << 8) | LSB;
  int32_t three = (LSB2 << 16) | two ;
  int32_t four = (MSB2 << 24) | three ;

  //Serial.println(four, DEC);
  return four;
}
