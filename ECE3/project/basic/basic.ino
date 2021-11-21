// Base code.
// 
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and 
// *    --blink the right front mainboard LED.
// *    
// *  You will need to add more code to
// *  make the car do anything useful. 
// 

//#include <ECE3_LCD7.h>

//uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;


const int right_nslp_pin=11;
const int right_dir_pin=30; //direction of right motor
const int right_pwm_pin=39;  //controls speed via variable voltage output

const int LED_RF = 41;
const int LED_LF = 51;

int startTime, currentTime;

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
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

  pinMode(LED_RF, OUTPUT);
  pinMode(LED_LF, OUTPUT);

  startTime = millis();
  
  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
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


void loop() {
  // put your main code here, to run repeatedly: 
  int leftSpd = 70;
  int rightSpd = 70;

//  currentTime = millis();
//
//  //goes forward
//  while (currentTime-startTime < 8000){
//    analogWrite(left_pwm_pin,leftSpd);
//    analogWrite(right_pwm_pin, rightSpd);
//    currentTime = millis();
//  }
//
//  startTime=millis();
//  
//  //does a 180
//  while (currentTime-startTime < 900)
//  {
//    digitalWrite(right_dir_pin, HIGH); //switch direction of right wheel
//    analogWrite(left_pwm_pin,leftSpd);
//    analogWrite(right_pwm_pin, rightSpd);
//    currentTime = millis();
//  }
//
//  startTime=millis();
//  //goes back
//  while (currentTime-startTime < 8000){
//    digitalWrite(right_dir_pin, LOW);
//    analogWrite(left_pwm_pin,leftSpd);
//    analogWrite(right_pwm_pin, rightSpd);
//    currentTime = millis();
//  }
//
//
//  startTime=millis();
//  //does a 180
//  while (currentTime-startTime < 900)
//  {
//    digitalWrite(right_dir_pin, HIGH); //switch direction of right wheel
//    analogWrite(left_pwm_pin,leftSpd);
//    analogWrite(right_pwm_pin, rightSpd);
//    currentTime = millis();
//  }
//  startTime=millis();
//  analogWrite(right_dir_pin,LOW);
//

  
  ECE3_read_IR(sensorValues);


  int modifySpd = 2 * map(errorFunction(sensorValues);

  
  

  digitalWrite(LED_RF, HIGH);
  digitalWrite(LED_LF, LOW);
  delay(250);
  digitalWrite(LED_RF, LOW);
  digitalWrite(LED_LF, HIGH);
  delay(250);
    
  }
  
