float r = 0.0825;
int sum_enc = 16000;
float rate_enc = (0.5) / sum_enc;

int encoderLA = 2;
int encoderLB = 3;
int encoderRA = 18;
int encoderRB = 19;
float D = 0.3;
float d = D/2;
int ln = 26560;
int rn = 24640;
int a = 0;

float inc_encL;
float enc_newL = 0;
long enc_oldL = 0;
float inc_encR;
float enc_newR = 0;
float enc_oldR = 0;
float length_error;

float ang_z = 0.0;
float ang_z_error;
float Xw = 0.0;
float Yw = 0.0;
float delta_d;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

bool bre = false;

//timer4 will interrupt at 1Hz
long variable = 200;
//storage variables
int c = 200;
void setup(){
Serial.begin(9600);
cli();//stop interrupts

//set timer4 interrupt at 1Hz
 TCCR4A = 0;// set entire TCCR1A register to 0
 TCCR4B = 0;// same for TCCR1B
 TCNT4  = 0;//initialize counter value to 0
 // set compare match register for 1hz increments
 OCR4A = 15625 ;// = (16*10^6) / (1*1024) - 1 (must be <65536)
 // turn on CTC mode
 TCCR4B |= (1 << WGM12);
 // Set CS12 and CS10 bits for 1024 prescaler
 TCCR4B |= (1 << CS12) | (1 << CS10);  
 // enable timer compare interrupt
 TIMSK4 |= (1 << OCIE4A);

sei();//allow interrupts
  
  pinMode(encoderLB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderRA, INPUT);
  pinMode(7,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderLB), updateEncoderR, RISING);
 // attachInterrupt(digitalPinToInterrupt(encoderRA), updateEncoderR, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(encoderRB), updateEncoderR, CHANGE);
}

ISR(TIMER4_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
   inc_encL = enc_newL - enc_oldL;
   inc_encR = enc_newR - enc_oldR;

   length_error = (inc_encR - inc_encL) * rate_enc;

   ang_z_error = length_error/(D);
   ang_z = ang_z + ang_z_error;

   delta_d = ((inc_encL + inc_encR)/2) * rate_enc;

   Xw = Xw + (delta_d * cos(ang_z));
   Yw = Yw + (delta_d * sin(ang_z));

   enc_oldL = enc_newL;
   enc_oldR = enc_newR;
   a++;
}


void loop(){
  if(ang_z < -0.05){
    //Serial.println(rate_enc, 8);
    Serial.println(inc_encL);
    Serial.println(inc_encR);
    //Serial.println(cos(ang_z),5);
    Serial.println(sin(delta_d),5);
    Serial.println(length_error);   
    Serial.println(Xw);
    Serial.println(Yw,5);
    Serial.println(ang_z);
    Serial.print("--- \n");
    Serial.println(a);
delay(10);
  }
    
    

  
}
void updateEncoderL(){

  enc_newL++;
}
void updateEncoderR(){
enc_newR++;
}
