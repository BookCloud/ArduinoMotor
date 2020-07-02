float Dbase = 0.3;
float dbase = Dbase/2;
float Dwheel = 0.165;
float rwheel = Dwheel/2;
int ppr = 800;
float Vright;
float Vleft;
int sum_enc = 16000;

int Fl;
int Fr;
float vel_x = -0.8;
float ang_z = -0.8;

void setup() {

  Serial.begin(9600);
  
  
}

void loop() {

Vleft = vel_x + ang_z * dbase;
Vright = vel_x - ang_z * dbase;

Fl = ppr * Vleft /(2*PI*rwheel);
Fr = ppr * Vright /(2*PI*rwheel);

Serial.print("left freq is;");
Serial.println(Fl);
Serial.print("right freq is;");
Serial.println(Fr);
Serial.println();

delay(1000);
}
