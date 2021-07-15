//
//Statorworks 2020
//Arduino example for SWI2C board servo control
/* 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
//
//Requirements:
//-I2CServo.ino, I2CWrapper.ino, OledSSD1306_Nano.ino on the sketch folder
//-Select your board on the i2CWrapper.ino file
//-If your Arduino board runs on 5V, don't forget to put an i2c level 
// shifter or repeater to interface the servo with 3.3V

#include "FABRIK2D.h"
#include "math.h"

// Arm Dimensions
#define BASE_HGT 125            // base hight
#define HUMERUS 325             // shoulder-to-elbow "bone"
#define ULNA 290                // elbow-to-wrist "bone"
#define GRIPPER 10               // gripper (incl. gimball and canera )

// Define boundaries in mm
#define X_MAX 300
#define X_MIN -300
#define Y_MAX 300
#define Y_MIN 0
#define Z_MAX 300
#define Z_MIN 0

#define RAD_TO_DEG  57296 / 1000
#define DEG_TO_RAD  1000 / 57296

int x[4] = {100,500,100,500};
int y[4] = {100,100,100,100};

/*
int x[1] = {290};
int y[1] = {325};
*/

int indexPos = 0; 

int lengths[] = {HUMERUS, ULNA, GRIPPER}; // 2DOF
Fabrik2D fabrik2D(4, lengths); // This arm has 3 joints; one in the origin, the elbow and the end effector.

#define LED_PIN 13
byte data[16];  
byte baseServo = 0x25; //modify address as needed here. servo default is 0x00
//byte shoulderServo = 0x65; //modify address as needed here. servo default is 0x00
byte shoulderServo2 = 0x22; // Double servo
byte elbowServo = 0x26; //modify address as needed here. servo default is 0x00
byte toolServoTilt = 0x65;

bool result;

// Ranges for TGY-S901D
int MAX_TRAVEL = 974; // Before endstop
int MIN_TRAVEL = 50; // Before Endstop

int TRAVEL_0 = 922; //0-ish deg
int TRAVEL_180 = 80; // 180-ish deg
int MID_TRAVEL = 512; // 90-ish deg

// 883 - 470 - 73
// 89 - 487 - 908
int DUAL_SERVO_ONE_MAX_TRAVEL = 914;
int DUAL_SERVO_ONE_MID_TRAVEL = 470;
int DUAL_SERVO_ONE_MIN_TRAVEL = 73;
int DUAL_SERVO_TWO_MAX_TRAVEL = 89;
int DUAL_SERVO_TWO_MID_TRAVEL = 487;
int DUAL_SERVO_TWO_MIN_TRAVEL = 908;

byte count=0;

void setup(){

 //Heartbeat LED
 pinMode(LED_PIN, OUTPUT); 
 digitalWrite(LED_PIN, HIGH);

 //I2C
 I2CWrapper_Begin();  //please select your board type in I2CWrapper.ino

 //console
 Serial.begin(9600); //Nano

fabrik2D.setTolerance(1);
 
 //Servo
 I2CServo_Begin();
 //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
 //I2CServo_Setup(i2c_address2, 2, 1, 0, 10, 60, 300, 50, 0);
 // I2CServo_SetTargetPosition(i2c_address2, 512, 0);
 //I2CServo_Setup(i2c_address3, 2, 1, 0, 10, 60, 300, 50, 0);
 // I2CServo_SetTargetPosition(i2c_address3, 512, 0);

 //I2CServo_Setup(shoulderServo2, 100, 1, 0, 200, 50, 1, 0, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
 //I2CServo_SetTargetPosition(shoulderServo2, DUAL_SERVO_TWO_MAX_TRAVEL, 0);

 //I2CServo_Setup(elbowServo, 100, 1, 0, 200, 50, 1, 0, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
 //I2CServo_SetTargetPosition(elbowServo, DUAL_SERVO_TWO_MAX_TRAVEL, 0);

 //I2CServo_Setup(shoulderServo2, 100, 1, 0, 200, 100, 1, 0, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
 //I2CServo_Setup(shoulderServo2, 100, 1, 0, 200, 100, 1, 0, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc

//I2CServo_Setup(shoulderServo2, 2, 1, 0, 100, 100, 500, 70, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
//I2CServo_Setup(elbowServo, 2, 1, 0, 100, 100, 500, 70, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
//I2CServo_Setup(toolServoTilt, 2, 1, 0, 100, 100, 500, 70, 0); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
delay(100);
// Scan for All servos
  for(byte i =0; i<=127;i++){
   if(I2CServo_GetI2cAddress(i, &data[0], false) ==1){
     Serial.print(i, HEX);
     Serial.print(" responded.\n");
     break;
   }
  }
}

void loop(){

int maxSpeed = 50;

I2CServo_Setup(shoulderServo2, 2, 1, 0, 100, maxSpeed, 500, 70, 1); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
I2CServo_Setup(elbowServo, 2, 1, 0, 100, maxSpeed, 500, 70, 1); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
I2CServo_Setup(toolServoTilt, 2, 1, 0, 100, maxSpeed, 500, 70, 1); //address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc

 //Flash heartbeat LED
 digitalWrite(LED_PIN, HIGH);//  //PORTB |= 1<<LED_PIN;
 delay(20); 
 digitalWrite(LED_PIN, LOW);//PORTB &= ~(1<<LED_PIN);
 delay(20);
    
  // Solve inverse kinematics given the coordinates x and y and the list of lengths for the arm.
  //bool res = fabrik2D.solve2(x, y, z, lengths);
  int xpos,ypos = 0;
 
  if(indexPos > 3) {
    indexPos = 0;
  }
  xpos = x[indexPos];
  ypos = y[indexPos];
  indexPos++;

  Serial.print("Step:");
  Serial.print(indexPos);
  Serial.print(" X: ");
  Serial.print(xpos);
  Serial.print(" Y: ");
  Serial.println(ypos);

 /*
  xpos = 200;
  ypos = 200;
  */
  
  bool solved = fabrik2D.solve(xpos, ypos, M_PI_2, lengths);
  
  if(solved)
    Serial.println("Solved!");
  else
    Serial.println("Not Solved!");  

  int angleSer1 = round(fabrik2D.getAngle(0)* RAD_TO_DEG);
  int angleSer2 = round(fabrik2D.getAngle(1)* RAD_TO_DEG);
  int toolAngleTilt = round(fabrik2D.getAngle(2)* RAD_TO_DEG);
  //int baseAngleDeg = round(fabrik2D.getBaseAngle()* RAD_TO_DEG);

  // Angles are printed in degrees.
  // The function calls below shows how easy it is to get the results from the inverse kinematics solution.

  int shoulderAngleCorrection = 0;
  int elbowAngleCorrection = 180;
  int angleSer1Calc = shoulderAngleCorrection + angleSer1;
  int angleSer2Calc = elbowAngleCorrection + angleSer2;       //abs(angleSer2);
  //Serial.print("Calculated Elbow Angle, Base Angle:");
  //Serial.print(res);
  Serial.print(", Tool: ");
  Serial.print(toolAngleTilt);
  Serial.print(", Shoulder Raw:");
  Serial.print(angleSer1);
  Serial.print(", Shoulder Calc:");
  Serial.print(angleSer1Calc);
  Serial.print(",  Elbow Raw:");
  Serial.print(angleSer2);
  Serial.print(",  Elbow Calc:");
  Serial.println(angleSer2Calc);
  //Serial.print(", ");
  //Serial.println(baseAngleDeg);

  Serial.print(fabrik2D.getX(0));
  Serial.print("\t");
  Serial.print(fabrik2D.getY(0));
  Serial.print("\t");
  Serial.print(fabrik2D.getX(1));
  Serial.print("\t");
  Serial.print(fabrik2D.getY(1));
  Serial.print("\t");
  Serial.print(fabrik2D.getX(2));
  Serial.print("\t");
  Serial.println(fabrik2D.getY(2));

  // Angles to set cannot be less then 0 and more tehn 180
  // No correction?
  // angleSer1 = constrain(angleSer1, 0, 180);
  angleSer1 = (min(180, max(0, angleSer1)));
  int shoulderTargetPosition = map(angleSer1, 0, 180, DUAL_SERVO_ONE_MIN_TRAVEL, DUAL_SERVO_ONE_MAX_TRAVEL);
  Serial.print("Shoulder Position to set: ");
  Serial.println(shoulderTargetPosition);
  I2CServo_SetTargetPosition(shoulderServo2, shoulderTargetPosition, 0);

  // Angles to set cannot be less then 0 and more tehn 180
  //angleSer2Calc = constrain(angleSer2Calc, 0, 180);
  angleSer2Calc = (min(180, max(0, angleSer2 + 180)));
  int elbowTargetPosition = map(angleSer2Calc, 0, 180, DUAL_SERVO_ONE_MIN_TRAVEL, DUAL_SERVO_ONE_MAX_TRAVEL);
  Serial.print("Elbow Position to set: ");
  Serial.println(elbowTargetPosition);
  bool resultElbow = I2CServo_SetTargetPosition(elbowServo, elbowTargetPosition, 0);
  Serial.print("Elbow Position set: ");  
  Serial.println(resultElbow);
  
  toolAngleTilt = (min(180, max(0, toolAngleTilt)));
  int toolTargetPosition = map(toolAngleTilt, 0, 180, DUAL_SERVO_ONE_MIN_TRAVEL, DUAL_SERVO_ONE_MAX_TRAVEL);
  Serial.print("Titlt Position to set: ");
  Serial.println(toolTargetPosition);
  I2CServo_SetTargetPosition(toolServoTilt, toolTargetPosition, 0);

  int16_t shoulderServo2pos;
  i2cServo_GetCurrentPosition(shoulderServo2, &shoulderServo2pos, 0);
  Serial.println(shoulderServo2pos);
  delay(200);

  int16_t elbowServo2pos;
  i2cServo_GetCurrentPosition(elbowServo, &elbowServo2pos, 0);
  Serial.println(elbowServo2pos);
  delay(200);


  delay(15000);
/*
 // Move
 int16_t pos2;
 int16_t pos3;

 // TODO: Translate solution angles to servo positions via MAP

 I2CServo_SetTargetPosition(i2c_address, MIN_TRAVEL, 0);
 I2CServo_SetTargetPosition(i2c_address2, DUAL_SERVO_ONE_MAX_TRAVEL, 0);
 I2CServo_SetTargetPosition(i2c_address3, DUAL_SERVO_TWO_MAX_TRAVEL, 0);
 I2CServo_SetTargetPosition(i2c_address3, DUAL_SERVO_TWO_MAX_TRAVEL, 0);
 
 delay(5000);
 i2cServo_GetCurrentPosition(i2c_address2, &pos2, 0);
 Serial.println(pos2);
 i2cServo_GetCurrentPosition(i2c_address3, &pos3, 0);
 Serial.println(pos3);
 delay(200);

 I2CServo_SetTargetPosition(i2c_address, MAX_TRAVEL, 0);
 I2CServo_SetTargetPosition(i2c_address2, DUAL_SERVO_ONE_MIN_TRAVEL, 0);
 I2CServo_SetTargetPosition(i2c_address3, DUAL_SERVO_TWO_MIN_TRAVEL, 0);

 delay(5000);
 i2cServo_GetCurrentPosition(i2c_address2, &pos2, 0);
 Serial.println(pos2);
 i2cServo_GetCurrentPosition(i2c_address3, &pos3, 0);
 Serial.println(pos3);
 delay(200); 
*/
 
}//loop
