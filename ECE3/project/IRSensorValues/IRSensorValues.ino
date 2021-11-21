#include <ECE3.h>

uint16_t sensorValues[8];

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;


const int right_nslp_pin=11;
const int right_dir_pin=30; //direction of right motor
const int right_pwm_pin=39;  //controls speed via variable voltage output

double error = 0;
double prevError = 0;
double prevError2 = 0;
double output;
double mapError1 = 0;
double mapError2 = 0;
int dark = 2200;




void setup() {
  // put your setup code here, to run once:
  ECE3_Init();

  //setup the motors
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);   //LOW means forward, HIGH means back
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);  //turns the motor on

   Serial.begin(9600);
  delay(2000);
}

int errorFunction(uint16_t sensorVals[])
{
  int s0 = -2.65*sensorVals[0];
  int s1 = -2.5*sensorVals[1];
  int s2 = -2*sensorVals[2];
  int s3 = -1*sensorVals[3];
  int s4 = sensorVals[4];
  int s5 = 2*sensorVals[5];
  int s6 = 2.5*sensorVals[6];
  int s7 = 2.65*sensorVals[7];
  return s0+s1+s2+s3+s4+s5+s6+s7;
}

void loop() {
  
  int leftSpd = 60;
  int rightSpd = 60; 
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues);
  error = errorFunction(sensorValues);


  mapError1 = map(error, -10000, 10000, -20, 20);

  //Serial.println(error); 

  output = 4 * (mapError1) +  8 * (mapError1 - prevError2);

//  Serial.println(output);
  Serial.println(mapError1-prevError2);
//  if(mapError1 < 0)
//  {
//    analogWrite(left_pwm_pin,leftSpd + abs(output));
//    analogWrite(right_pwm_pin, rightSpd - abs(output));
//  }
//  else if (mapError1 > 0)
//  {
//    analogWrite(left_pwm_pin,leftSpd - abs(output));
//    analogWrite(right_pwm_pin, rightSpd + abs(output));
//  }
//
//  if((sensorValues[0] >= dark) && (sensorValues[1] >= dark) && (sensorValues[2] >= dark) && (sensorValues[3] >= dark) 
//  && (sensorValues[4] >= dark) && (sensorValues[5] >= dark) && (sensorValues[6] >= dark) && (sensorValues[7] >= dark))
//  {
//    ECE3_read_IR(sensorValues);
//    if((sensorValues[0] >= dark) && (sensorValues[1] >= dark) && (sensorValues[2] >= dark) && (sensorValues[3] >= dark) 
//     && (sensorValues[4] >= dark) && (sensorValues[5] >= dark) && (sensorValues[6] >= dark) && (sensorValues[7] >= dark))
//    {
////      ECE3_read_IR(sensorValues);
////      if((sensorValues[0] >= dark) && (sensorValues[1] >= dark) && (sensorValues[2] >= dark) && (sensorValues[3] >= dark) 
////      && (sensorValues[4] >= dark) && (sensorValues[5] >= dark) && (sensorValues[6] >= dark) && (sensorValues[7] >= dark))
////      {
//         leftSpd = 100;
//         rightSpd = 100;
//         digitalWrite(right_dir_pin,HIGH);
//
//         analogWrite(left_pwm_pin,leftSpd);
//         analogWrite(right_pwm_pin,rightSpd);
//
//         delay(550);
//
//         digitalWrite(right_dir_pin, LOW);
//
//         analogWrite(left_pwm_pin,leftSpd);
//         analogWrite(right_pwm_pin,rightSpd);
//         delay(50);
//    
////      }
//      }
//  }
  prevError2 = prevError;
  prevError = mapError1;
  
 

  
}
