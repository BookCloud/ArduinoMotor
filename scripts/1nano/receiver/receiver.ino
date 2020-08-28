const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial.println("<Arduino is ready>");
}

void loop() {
    clearbuffer();    //remove old data from serial buffer
    recvBytesWithStartEndMarkers(); //receive newest data
    showNewData();    //process data
}

void clearbuffer(){
  //while something in serial buffer, read and throw away the data
  while(Serial2.available() >0){
    Serial2.read(); //do nothing with it
  }
}


void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;  // "<" hex to ascii
    byte endMarker = 0x3E;    // ">"
    byte rb;

    //wait for newest data to arrive
    while(Serial2.available() <= 0){
      int wait = 0; //do nothing here
    }
    // start reading
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
    //print the bytes received 1 by 1
    if (newData == true) {
        for (byte n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes[n], BIN );
            Serial.print(' ');
        }
        pars(); //put the seperate bytes back to one piece     
        newData = false;
    }
}

void pars(){
    //  sort out the seperate bytes (HARDCODED!!!)  first 4bytes belong to the first integer sent by nano
    int32_t p = combine(receivedBytes[0], receivedBytes[1], receivedBytes[2], receivedBytes[3] );
    int32_t q = combine(receivedBytes[4], receivedBytes[5], receivedBytes[6], receivedBytes[7] );

    Serial.println();
    Serial.print("Parsed: ");
    Serial.print(p);
    Serial.print(" ");
    Serial.println(q);

}

//shifts the four 8bits back into a 32bit integer
int32_t combine(uint32_t LSB, uint32_t MSB, uint32_t LSB2, uint32_t MSB2){
  int32_t two = (MSB << 8) | LSB;
  int32_t three = (LSB2 << 16) | two ;
  int32_t four = (MSB2 << 24) | three ;

  //Serial.println(four, DEC);
  return four;
}
