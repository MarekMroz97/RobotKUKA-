
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

//double dt = 0.001;

MPU6050 accelgyro;
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

/*int16_t ax0, ay0, az0;  // define accel as ax,ay,az
int16_t gx0, gy0, gz0;   */// define gyro as gx,gy,gz
String inputString = ""; // empty string
bool stringComplete = false;
/*
bool firstmeasurment = false;
double calkax1,calkax2, calkay1, calkay2, calkaz1,calkaz2;
double pomx, pomy, pomz;
double anglex, angley, anglez;
double x,y,z;


// pierdoły do filtracji
double a11 = 1;
double a12 = -dt;
double a21 = 0;
double a22 = 1;
double b1 = dt;
double b2 = 0;
double c1 = 1;
double c2 = 0;
double x01 = 0.0;
double x02 = 0.0;
double P011 = 1.0;
double P012 = 0.0;
double P021 = 0.0;
double P022 = 1.0;
double xpri = x0;
double Ppri = P0;
double xpost = x0;
double Ppost = P0;
double noise1 = 1; //szum pomiarowy
double noise2 = 2; //szum procesowy
double V11 = noise1*noise1*dt;
double V12 = 0;
double V21 = 0;
double V22 =  noise1*noise1*dt;
double W = noise2*noise2;


void Kalman(){

  
}

*/
void setup() {
  Wire.begin();      // join I2C bus
  Serial.begin(9600);    //  initialize serial communication
// Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
pinMode(7, INPUT_PULLUP);
  // verify connection
//Serial.println("Testing device connections...");
//Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}
/*
void integral(double *currentvalue, double *calka1, double *calka2,double dx, double dt){
  double calkapom1 = *calka1;
  double calkapom2 = *calka2;
  *calka1 = calkapom1 + dt*(*currentvalue + dx)/2;
  *calka2 = calkapom2 + dt*(calkapom1+*calka1)/2;
  *currentvalue = dx;
  
}*/
void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  /*
if(firstmeasurment == false){
  Serial.print("X=");
  Serial.print(ax/16384.0);
  Serial.print("|Y=");
  Serial.print(ay/16384.0);
  Serial.print("|Z=");
  Serial.println(az/16384.0);
if(ax/16384.0 > 1.0){anglex = asin(1.0);}
else if(ax/16384.0 < -1.0){anglex = asin(-1.0);}
else{anglex = asin(ax/16384.0);}
if(ay/16384.0 > 1.0){angley = asin(1.0);}
else if(ay/16384.0 < -1.0){angley = asin(-1.0);}
else{angley = asin(ay/16384.0);}
if(az/16384.0 > 1.0){anglez = asin(1.0);}
else if(az/16384.0 < -1.0){anglez = asin(-1.0);}
else{anglez = asin(az/16384.0);}
anglex = anglex*180.0 / PI;
angley = angley*180.0 / PI;
anglez = anglez*180.0 / PI;
Serial.println(anglex);
Serial.println(angley);
Serial.println(anglez);
firstmeasurment = true;

calkax2 = anglex;
calkay2 = angley;
calkaz2 = anglez;
calkax1 = 0.0;
calkay1 = 0.0;
calkaz1 = 0.0;
x =0.0;
y =0.0;
z =0.0;
}


calkax1 = 0.0;
calkay1 = 0.0;
calkaz1 = 0.0;
  integral(&x, &calkax1, &calkax2, gx/131.0, dt);
  integral(&y, &calkay1, &calkay2, gy/131.0, dt);
  integral(&z, &calkaz1, &calkaz2, gz/131.0, dt);
  Serial.print("aX=");
  Serial.print(calkax2);
  Serial.print(" aY=");
  Serial.print(calkay2);
  Serial.print(" aZ=");
  Serial.println(calkaz2);
*/
if(stringComplete == true){
if(digitalRead(7) == LOW){  
  
  Serial.print("X=");
//Serial.print(ax/16384.0 - sin(anglex));
  Serial.print(ax/16384.0);
  Serial.print("|Y=");
 //erial.print(ay/16384.0 - sin(angley));
  Serial.print(ay/16384.0);
  Serial.print("|Z=");
 //erial.print(az/16384.0 - sin(anglez));
  Serial.println(az/16384.0);
  //Serial.print(" | Gir. X = ");
// Serial.print(gx/131.0);
// Serial.print(" | Y = ");
 // Serial.print(gy/131.0);
 // Serial.print(" | Z = ");
//Serial.println(gz/131.0);
//Serial.print(" aX=");
  //Serial.print(sin(anglex));
  //Serial.print("|aY=");
  //Serial.print(sin(angley));
  //Serial.print("|aZ=");
  //Serial.println(sin(anglez));
}
else{
  Serial.println("X=0.00|Y=0.00|Z=0.00");/* | Gir. X = 0.00 | Y = 0.00 | Z = 0.00");*/
  
}
stringComplete = false;
inputString = "";
}
//delay(1000*dt);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}



