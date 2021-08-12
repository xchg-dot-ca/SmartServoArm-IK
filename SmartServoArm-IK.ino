#include "FABRIK2D.h"
#include "math.h"

// Arm Dimensions
#define BASE_HGT 125            // base hight
#define HUMERUS 325             // shoulder-to-elbow "bone"
#define ULNA 290                // elbow-to-wrist "bone"
#define GRIPPER 10              // gripper (incl. gimball and canera )

// Define boundaries in mm
#define X_MAX 300
#define X_MIN -300
#define Y_MAX 300
#define Y_MIN 0
#define Z_MAX 300
#define Z_MIN 0

// Define path of the camera
int x[4] = {100,500,100,500};
int y[4] = {100,100,100,100};

// Number of steps for path to be divided
const int steps = 100;
// Time between steps, ms
int timeBetweenSteps = 2000;
int timedSteps[steps];

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

// Values very specific to the shoulder double servo ( 180 deg range )
int DUAL_SERVO_DIRECT_MAX_TRAVEL = 926;   //( 180 deg )
int DUAL_SERVO_DIRECT_MID_TRAVEL = 500;   //( 90 deg )
int DUAL_SERVO_DIRECT_MIN_TRAVEL = 103;    //( 0 deg )
int DUAL_SERVO_REVERSE_MAX_TRAVEL = 95;  //( 180 deg )
int DUAL_SERVO_REVERSE_MID_TRAVEL = 508; //( 90 deg )
int DUAL_SERVO_REVERSE_MIN_TRAVEL = 918; //( 0 deg )

// Values very specific to the elbow servo ( 180 deg range )
int ELBOW_SERVO_MAX_TRAVEL = 897; //( 180 deg )
int ELBOW_SERVO_MID_TRAVEL = 504; //( 90 deg )
int ELBOW_SERVO_MIN_TRAVEL = 81; //( 0 deg )

// Values very specific to the tool servo ( 180 deg range )
int TOOL_SERVO_MAX_TRAVEL = 928; //( 180 deg )
int TOOL_SERVO_MID_TRAVEL = 529; //( 90 deg )
int TOOL_SERVO_MIN_TRAVEL = 109; //( 0 deg )

byte count = 0;

int maxSpeed = 200;

int xpos = 0;
int ypos = 0;

void setup(){

 //Heartbeat LED
 pinMode(LED_PIN, OUTPUT); 
 digitalWrite(LED_PIN, HIGH);

 //I2C
 I2CWrapper_Begin();  //please select your board type in I2CWrapper.ino

 //Console
 Serial.begin(115200);
 delay(1000);
 
 fabrik2D.setTolerance(1);
 
 //Servo
 I2CServo_Begin();
  delay(100);
  
  // Scan for All servos
  for(byte i =0; i<=127;i++){
   if(I2CServo_GetI2cAddress(i, &data[0], false) ==1){
     Serial.print(i, HEX);
     Serial.print(" responded.\n");
     break;
   }
  }

  int distanceX = x[1] - x[0];
  int distancePerStep = distanceX/steps;
  // Prepare timed steps
  for(byte i = 0; i < steps; i++){
    timedSteps[i] = x[0] + (i * distancePerStep);
    Serial.print(timedSteps[i]);
    Serial.println(" , Next Step");
    delay(100);
  }
}

bool thatway = true;

void loop(){

  // Set Servos operationa parameters
  // address, park_type, motor_polarity, continuous_rotation, max_power, max_speed, ramp_time ms, ramp_curve, use_crc
  I2CServo_Setup(shoulderServo2, 2, 1, 0, 100, maxSpeed, 500, 70, 1);
  I2CServo_Setup(elbowServo, 2, 1, 0, 100, maxSpeed, 500, 70, 1);
  I2CServo_Setup(toolServoTilt, 2, 1, 0, 100, maxSpeed, 500, 70, 1);

/*
  if(indexPos > steps) {
    indexPos = 0;
  }
*/
  if( indexPos < steps && thatway) {
    xpos = timedSteps[indexPos];
    ypos = 100;
    indexPos++;
  } else if(thatway) {
    thatway = false;
  }

  if( indexPos > 0 && !thatway) {
    xpos = timedSteps[indexPos];
    ypos = 100;
    indexPos--;
  } else if(!thatway) {
    thatway = true;
  }
  
  //indexPos++;

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

  // Move the arm with tool angle set to 90deg, 
  // TBD: The only thing is I cannot wrap my mind 90deg in relation to what ?
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
  int shoulderTargetPosition = map(angleSer1, 0, 180, DUAL_SERVO_DIRECT_MIN_TRAVEL, DUAL_SERVO_DIRECT_MAX_TRAVEL);
  Serial.print("Shoulder Position to set: ");
  Serial.println(shoulderTargetPosition);
  I2CServo_SetTargetPosition(shoulderServo2, shoulderTargetPosition, 0);

  //Angles to set cannot be less then 0 and more tehn 180
  //angleSer2Calc = constrain(angleSer2Calc, 0, 180);
  angleSer2Calc = (min(180, max(0, angleSer2 + 180)));
  int elbowTargetPosition = map(angleSer2Calc, 0, 180, ELBOW_SERVO_MIN_TRAVEL, ELBOW_SERVO_MAX_TRAVEL);
  Serial.print("Elbow Position to set: ");
  Serial.println(elbowTargetPosition);
  bool resultElbow = I2CServo_SetTargetPosition(elbowServo, elbowTargetPosition, 0);
  Serial.print("Elbow Position set: ");  
  Serial.println(resultElbow);

  /**
   * We need to compensate for the fact that Camera is connected to large the side of the servo, not small side
   * Hence we need to adjust for 90deg ( just need to figure out +90 or -90 :)
   */
  toolAngleTilt = (min(180, max(0, toolAngleTilt)));
  int toolTargetPosition = map(toolAngleTilt, 0, 180, TOOL_SERVO_MIN_TRAVEL, TOOL_SERVO_MAX_TRAVEL);
  // Hardcode tool angle to 90deg. or 512 value
  // toolTargetPosition = 512;
  Serial.print("Tilt Position to set: ");
  Serial.println(toolTargetPosition);
  I2CServo_SetTargetPosition(toolServoTilt, toolTargetPosition, 0);

  int16_t shoulderServo2pos = 0;
  i2cServo_GetCurrentPosition(shoulderServo2, &shoulderServo2pos, 0);
  Serial.println(shoulderServo2pos);
  delay(timeBetweenSteps);

  int16_t elbowServo2pos = 0;
  i2cServo_GetCurrentPosition(elbowServo, &elbowServo2pos, 0);
  Serial.println(elbowServo2pos);
  
  //delay(200);
  //delay(15000);
}//loop
