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
  
//  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  int leftSpd = 120;
  int rightSpd = 100;

  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);

  delay(8000);
  digitalWrite(right_dir_pin,HIGH);

  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);

  delay(750);
  digitalWrite(right_dir_pin,LOW);
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);
  delay(8000);

  digitalWrite(right_dir_pin,HIGH);
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);
  delay(750);

  digitalWrite(right_dir_pin,LOW);


  
  
    
  }
  
