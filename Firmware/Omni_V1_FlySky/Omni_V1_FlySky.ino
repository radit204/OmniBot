

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//-----( Import needed libraries )-----
//#include <SPI.h>
//#include <nRF24L01.h>
//#include <RF24.h>

#include "FlySkyIBus.h"
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN 6
#define CSN_PIN 8

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// NOTE: the "LL" at the end of the constant is "LongLong" type
//const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

#define MOT_FR_A  A0  //R_INA1
#define MOT_FR_B  A1  //R_INA2
#define MOT_BR_A  A2  //R_INB1
#define MOT_BR_B  A3  //R_INB2

#define MOT_FL_A  2  //L_INA1
#define MOT_FL_B  3  //L_INA2
#define MOT_BL_A  4  //L_INB1
#define MOT_BL_B  5  //L_INB2

#define MAXSPEED 4096 //4096 MAX

/*
 * SPI  :13,12,11,10
 * I2C  :A4,A5
 */

/*-----( Declare objects )-----*/
//RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/

//int joystick[6];  // 6 element array holding Joystick readings
int speedFR = 0;
int speedFL = 0;
int speedBR = 0;
int speedBL = 0;
int joy_y;
int joy_x;
int joy_r;
int x;
int y;
int r;
int i = 0;

// the four button variables from joystick
int buttonUp;
int buttonRight;
int buttonDown;
int buttonLeft;

void setup()
{
  pwm.begin();
  pinMode(MOT_FR_A, OUTPUT);
  pinMode(MOT_FR_B, OUTPUT);
  pinMode(MOT_BR_A, OUTPUT);
  pinMode(MOT_BR_B, OUTPUT);

  pinMode(MOT_FL_A, OUTPUT);
  pinMode(MOT_FL_B, OUTPUT);
  pinMode(MOT_BL_A, OUTPUT);
  pinMode(MOT_BL_B, OUTPUT);

  IBus.begin(Serial);

  Serial.begin(115200);
  //  Serial.println("Nrf24L01 Receiver Starting");

  //  radio.begin();
  //  radio.openReadingPipe(1, pipe);
  //  radio.startListening();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(330);  // This is the maximum PWM frequency

  Wire.setClock(400000);
}


void loop() {

  // Y-axis used for forward and backward control
  // X-axis used for left and right control

  //Flysky FS-I6 + RX2A Pro V1 (Over IBUS)
  // Min value = 1000, center value =1500, max value = 1500
  //Channels
  //Channel 0 = Right Y
  //Channel 1 = Right X
  //Channel 2 = Left Y
  //Channel 3 = Left X
  //Channel 4 = SWA
  //Channel 5 = SWB
  //Channel 6 = VarA
  //Channel 7 = VarB
  //Channel 8 = SWC
  //Channel 9 = SWD



  IBus.loop();
  joy_x = IBus.readChannel(0);
  joy_y = IBus.readChannel(1);
  joy_r = IBus.readChannel(3);

  // Biar ga langsung muter pas pertama dinyalain
  if (joy_x == 0) {
    joy_x = 1500;
    joy_y = 1500;
    joy_r = 1500;
  }

  //Joystick X Axis mapping
  if (joy_x <= 1500) {
    x = map(joy_x, 1500, 1000, 0, MAXSPEED);
  }
  else if (joy_x >= 1500) {
    x = map(joy_x, 1500, 2000, 0, -MAXSPEED);
  }
  else {
    x = 0;
  }

  //Joystick Y Axis mapping
  if (joy_y <= 1500) {
    y = map(joy_y, 1500, 1000, 0, MAXSPEED);
  }
  else if (joy_y >= 1500) {
    y = map(joy_y, 1500, 2000, 0, -MAXSPEED);
  }
  else {
    y   = 0;
  }

  //Joystick LY Axis mapping
  if (joy_r <= 1500) {
    r = map(joy_r, 1500, 1000, 0, MAXSPEED);
  }
  else if (joy_r >= 1500) {
    r = map(joy_r, 1500, 2000, 0, -MAXSPEED);
  }
  else {
    r   = 0;
  }

  //Holonomic Formula
  speedFR = (-y) + x + r;
  speedFL = y + x + r;
  speedBR = (-y) + (-x) + r;
  speedBL = y + (-x) + r;

  //Constrain Values
  speedFR = constrain(speedFR, -4095, 4095);
  speedFL = constrain(speedFL, -4095, 4095);
  speedBR = constrain(speedBR, -4095, 4095);
  speedBL = constrain(speedBL, -4095, 4095);

  //Set Motor Directions

  //---FR MOTOR---
  if (speedFR < 0) {
    digitalWrite(MOT_FR_A, LOW);
    digitalWrite(MOT_FR_B, HIGH);
  }
  else if (speedFR >= 0) {
    digitalWrite(MOT_FR_A, HIGH);
    digitalWrite(MOT_FR_B, LOW);
  }
  //    else {
  //      digitalWrite(MOT_FR_A, LOW);
  //      digitalWrite(MOT_FR_B, LOW);
  //    }

  //---FL MOTOR---
  if (speedFL < 0) {
    digitalWrite(MOT_FL_A, LOW);
    digitalWrite(MOT_FL_B, HIGH);
  }
  else if (speedFL >= 0) {
    digitalWrite(MOT_FL_A, HIGH);
    digitalWrite(MOT_FL_B, LOW);
  }

  //    else {
  //      digitalWrite(MOT_FL_A, LOW);
  //      digitalWrite(MOT_FL_B, LOW);
  //    }

  //---BR MOTOR---
  if (speedBR < 0) {
    digitalWrite(MOT_BR_A, HIGH);
    digitalWrite(MOT_BR_B, LOW);
  }
  else if (speedBR >= 0) {
    digitalWrite(MOT_BR_A, LOW);
    digitalWrite(MOT_BR_B, HIGH);
  }

  //    else {
  //      digitalWrite(MOT_BR_A, LOW);
  //      digitalWrite(MOT_BR_B, LOW);
  //    }

  //---BL MOTOR---
  if (speedBL < 0) {
    digitalWrite(MOT_BL_A, HIGH);
    digitalWrite(MOT_BL_B, LOW);
  }
  else if (speedBL >= 0) {
    digitalWrite(MOT_BL_A, LOW);
    digitalWrite(MOT_BL_B, HIGH);
  }

  //    else {
  //      digitalWrite(MOT_BL_A, LOW);
  //      digitalWrite(MOT_BL_B, LOW);
  //    }

  speedFR = abs(speedFR);
  speedFL = abs(speedFL);
  speedBR = abs(speedBR);
  speedBL = abs(speedBL);

  pwm.setPWM(3, 0, speedFR );
  pwm.setPWM(0, 0, speedFL );
  pwm.setPWM(2, 0, speedBR );
  pwm.setPWM(1, 0, speedBL );

//    For Debugging
//    Serial.print("<");
//    Serial.print(speedFR);
//    Serial.print(",");
//    Serial.print(speedFL);
//    Serial.print(",");
//    Serial.print(speedBR);
//    Serial.print(",");
//    Serial.print(speedBL);
//    Serial.print(",");
//    Serial.print(joy_x);
//    Serial.print(",");
//    Serial.print(joy_y);
//    Serial.println(">");

}
