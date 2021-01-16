#include "Wire.h"
#include "MPU9250.h"

//Calculating the calibrated magnetometer data in real time with using the matrix and the bias 
MPU9250 IMU(Wire,0x68);
int status;

float xv, yv, zv;

void setup() {
  // serial to display data
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  // wait for serial port to connect
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop()
{
  float values_from_magnetometer[3];
  float angle;
  
  IMU.readSensor();
  values_from_magnetometer[0] = IMU.getMagX_uT();
  values_from_magnetometer[1] = IMU.getMagY_uT();
  values_from_magnetometer[2] = IMU.getMagZ_uT();
  
  Serial.flush(); 
  Serial.print(values_from_magnetometer[0]); 
  Serial.print(",");
  Serial.print(values_from_magnetometer[1]);
  Serial.print(",");
  Serial.print(values_from_magnetometer[2]);
  Serial.print(",");
  Serial.println();

  delay(100); 
} 
 
