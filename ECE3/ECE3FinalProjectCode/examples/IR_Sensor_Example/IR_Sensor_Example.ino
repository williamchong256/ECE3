#include <ECE3.h>

uint16_t sensorValues[8];

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
}

int errorFunction(uint16_t sensorVals[])
{
  int s0 = -6*sensorVals[0];
  int s1 = -4*sensorVals[1];
  int s2 = -2*sensorVals[2];
  int s3 = -1*sensorVals[3];
  int s4 = sensorVals[4];
  int s5 = 2*sensorVals[5];
  int s6 = 4*sensorVals[6];
  int s7 = 6*sensorVals[7];
  return s0+s1+s2+s3+s4+s5+s6+s7;
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
//  for (unsigned char i = 0; i < 8; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//    
//  }
  Serial.print(errorFunction(sensorValues));
  Serial.println();

  delay(50);
}
