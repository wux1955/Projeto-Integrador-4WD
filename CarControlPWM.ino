//Car Control and Ultrasonic-Display
#include <car_bluetooth.h>
#include <SoftwareSerial.h>   //Software Serial Port
#include <String.h>
#include "MotorDriver.h"
#include <Ultrasonic.h>
#include "TM1637.h"

#define RxD 2//D2 of Arduino should connect to TX of the Serial Bluetooth module
#define TxD 4//D4 of Arduino should connect to RX of the Serial Bluetooth module
CarBluetooth bluetooth(RxD, TxD);
#define CMD_INVALID     0XFF
#define CMD_FORWARD     'F'
#define CMD_RIGHT_FRONT 'R'

#define CMD_BACKWARD    'B'

#define CMD_LEFT_FRONT  'L'
#define CMD_STOP        'S'

#define SPEED_STEPS 20
uint8_t speed0 = 100;

//--------------------------------------
//Ultrasonic
#define TRIGGER_PIN  5//connect Trip of the Ultrasonic Sensor moudle to D5 of Arduino 
#define ECHO_PIN     3
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

#define CLK A5//connect CLK of the 4-Digit Display to A5 of Arduino and can be changed to other ports    
#define DIO A4//
TM1637 disp(CLK,DIO);

float cmMsec, inMsec;

//--------------------------------------
//PWM speed control
#define MIN_DIST 10 //Minimum needed reading value of ultrassonic sensor (>0)
#define MAX_DIST 400 //Maximum reading value of ultrassonic sensor
//#define enA 9 //Enable Pin of motor A
//#define enB 8 //Enable Pin of motor B

//--------------------------------------

void setup(){
  disp.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  disp.init();
  motordriver.init();
  motordriver.setSpeed(100,MOTORA);
  motordriver.setSpeed(100,MOTORB);
  bluetooth.waitConnected();

  //PWM speed control
  //pinMode(enA, OUTPUT);
  //pinMode(enB, OUTPUT);

}

uint8_t bt_command;

#define CAR_STOP 0
#define CAR_FORWARD 1
#define CAR_BACK 2
uint8_t car_status = CAR_STOP;
uint8_t new_status = car_status;

void loop(){
  ultrasonicDisplay();
  bt_command = bluetooth.readByte();
  if(bt_command != CMD_INVALID){
  controlCar(bt_command);
  }

}


void controlCar(uint8_t cmd){
  int pwmOutput = map(distance, MIN_DIST, MAX_DIST, 0, 255)
  switch(cmd)
  {
    case CMD_FORWARD:     
      motordriver.goForward();
        //PWM Control
        if (cmMsec < MAX_DIST){
          motordriver.setSpeed(pwmOutput,MOTORA);
          motordriver.setSpeed(pwmOutput,MOTORB);
        }else{
          motordriver.setSpeed(speed0,MOTORA);
          motordriver.setSpeed(speed0,MOTORB);
        }
      break;
    case CMD_RIGHT_FRONT: 
      //  if(car_status != CAR_STOP)new_status = CAR_FORWARD;
      motordriver.goRight();
      //PWM Control
      if (cmMsec < MAX_DIST){
        motordriver.setSpeed(pwmOutput,MOTORA);
        motordriver.setSpeed(pwmOutput,MOTORB);
      }else{
        motordriver.setSpeed(speed0,MOTORA);
        motordriver.setSpeed(speed0,MOTORB);
      }
      // delay(200); 
      break;
    case CMD_BACKWARD:    
      motordriver.goBackward();
      break;
    case CMD_LEFT_FRONT:
      motordriver.goLeft();
      //PWM Control
      if (cmMsec < MAX_DIST){
        motordriver.setSpeed(pwmOutput,MOTORA);
        motordriver.setSpeed(pwmOutput,MOTORB);
      }else{
        motordriver.setSpeed(speed0,MOTORA);
        motordriver.setSpeed(speed0,MOTORB);
      }
      // delay(200);
      break;
    case CMD_STOP:
      motordriver.stop();
      break;
    default:
      break;
  }
  if((cmd>='0')&&(cmd<='9'))
  {
  	speed0 = cmd-0x30;
    Serial.print(speed0);
    Serial.print(">");
    speed0 = map(speed0, 0, 9, 0, 255);
    Serial.println(speed0);
    motordriver.setSpeed(speed0,MOTORA);
    motordriver.setSpeed(speed0,MOTORB);
  }
}

void speedUp(){
  if(speed0 < 236)speed0 += SPEED_STEPS;
  else speed0 = 255;
  motordriver.setSpeed(speed0,MOTORA);
  motordriver.setSpeed(speed0,MOTORB);
}

void speedDown(){
  if(speed0 > 70)speed0 -= SPEED_STEPS;
  else speed0 = 50;
  motordriver.setSpeed(speed0,MOTORA);
  motordriver.setSpeed(speed0,MOTORB);
}
void ultrasonicDisplay()
{
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  inMsec = ultrasonic.convert(microsec, Ultrasonic::IN);
  disp.display((int16_t)cmMsec);//in centimeters
}
