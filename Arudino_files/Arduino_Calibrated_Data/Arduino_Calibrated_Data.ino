#include "Wire.h"
#include "MPU9250.h"

//Calculating the calibrated magnetometer data in real time with using the matrix and the bias 
MPU9250 IMU(Wire,0x68);
int status;

float xv, yv, zv;

//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibrated_values[3];   
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  double calibration_matrix[3][3] = 
  {
    {0.02527556, -0.00014847, -0.00093046},
    {-0.00014847, 0.02555101, -0.00057069},
    {-0.00093046, -0.00057069, 0.02781363}  
  };
  
  //bias[3] is the bias
  double bias[3] = 
  {
    21.39111575,
     -8.59021922,
    -31.94676112
  };  
  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

float northDeclination(float calibratedX, float calibratedY, float calibratedZ) {
  return atan2(calibratedY, calibratedX);
}

double calculateAbsoluteMfieldValue(float calibratedX, float calibratedY, float calibratedZ) {
  float x = pow(calibratedX, 2);
  float y = pow(calibratedY, 2);
  float z = pow(calibratedZ, 2);
  
  return sqrt(x + y + z);
}

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
  double absoluteMfieldValue;
  double prov;
  
  IMU.readSensor();
  values_from_magnetometer[0] = IMU.getMagX_uT();
  values_from_magnetometer[1] = IMU.getMagY_uT();
  values_from_magnetometer[2] = IMU.getMagZ_uT();
  transformation(values_from_magnetometer);
  
  Serial.flush(); 
  Serial.print(values_from_magnetometer[0]); 
  Serial.print(",");
  Serial.print(values_from_magnetometer[1]);
  Serial.print(",");
  Serial.print(values_from_magnetometer[2]);
  
  Serial.print(",");
  angle = northDeclination(calibrated_values[0], calibrated_values[1], calibrated_values[2]);
  Serial.print(angle*180/3.14);
  
  Serial.print(",");
  absoluteMfieldValue = calculateAbsoluteMfieldValue(calibrated_values[0], calibrated_values[1], calibrated_values[2]);
  Serial.print(absoluteMfieldValue);
   
  Serial.println();

  delay(100); 
} 
 
