/* ************************************************************************************************* */
// * ECE 201: Line Following Robot with PID + Safety Logic * //
/* ************************************************************************************************* */

// ************************************************************************************************* //
// Declare Variables

// Variables and Libaries for Motor
#include <Wire.h>
#include <Adafruit_MotorShield.h> 

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Motors can be switched here (1) <--> (2)
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // left motor
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2); // right motor
  
// Set Initial Speed of Motors (CAN BE EDITED BY USER)
// Default speed set to 60 as recommended
const int M1Sp = 60; 
const int M2Sp = 60;

// Variables for Potentiometer
// Assigned to Analog pins A0-A3
const int S_pin = A0; //proportional control
const int P_pin = A1; //proportional control
const int I_pin = A2; //integral control
const int D_pin = A3; //derivative control

// User set SPID values
// These are the MAXIMUM values that the potentiometers can map to.
// Tuned for standard line following behavior.
const float speed       = 140; // Max added speed
const float proportion  = 80;  // Max Kp
const float integral    = 5;   // Max Ki (Keep small to prevent windup)
const float derivative  = 80;  // Max Kd

const int numMeas = 30; // setting numMeas to 40 as in code 10.4 is recommended

// Initialize SPID values to zero
int SpRead = 0; //Speed Increase 
int kPRead = 0; //proportional gain
int kIRead = 0; //integral gain
int kDRead = 0; //derivative gain
  
// Variables for Light Sensors
// LDR_Pin holds 7 values of the LDR pin connection to the Arduino
int LDR_Pin[7] = {A8, A9, A10, A11, A12, A13, A14}; 
int LDR[7]; // this is an empty array that will hold the readings from our potentiometers

// LED Pin (PRESET DO NOT CHANGE)
int led_Pin = 32; 

// Calibration Variables
float Mn[7]; // array containing minimum read values of every potentiometer
float Mx[7]; // array containing maximum read values of every potentiometer
float LDRf[7] = {0.,0.,0.,0.,0.,0.,0.}; // initialize the LDR read values to float zero

int MxRead; // maximum read value by any potentiometer
int MxIndex; // index (location in array) of max read value
float AveRead; // average read value of potentiometer
int CriteriaForMax; // value to determine whether a read value is a max
float WeightedAve; // weighted average after calibration
  
// Error calculation variables
int im0, im1, im2; 
  
// Motor control variables
int M1SpeedtoMotor, M2SpeedtoMotor; // actual speeds motor will be rotating at
int Turn; // which direction and how sharply should the cart turn
int M1P = 0, M2P = 0; // proportion control for motors 1 and 2 respectively
float error; // current error
int lasterror = 0; // previous error
int sumerror = 0; // sum of total errors
float kP, kI, kD; // final values of P, I, and D used in control
 
// ************************************************************************************************* //
// setup - runs once
void setup() {
  Serial.begin(9600); // Standard baud rate
  AFMS.begin(); // initialize the motor

  pinMode(led_Pin, OUTPUT); // set the led_pin to be an output
  
  Calibrate(); // Calibrate black and white sensing
  
  ReadPotentiometers(); // Read potentiometer values (Sp, P, I, & D)
  
  delay(2000); // Delay to ensure cart is properly placed before moving
  
  RunMotors(); // Starts motors straight forward
 
} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {
  // Note: some functions are only helpful for debugging and can be commented out once functionality is ensured
 
  ReadPotentiometers(); // Only if you want to see Potentiometers working in set up as you run the line following
  
  ReadPhotoResistors(); // Read photoresistors and map to 0-100 based on calibration
  
  // *** SAFETY CHECK ***
  // If CheckSafety returns false, it means danger is detected.
  // We stop the motors and skip the PID calculation.
  if (CheckSafety() == false) {
    EmergencyStop();
    // Do NOT proceed to CalcError or RunMotors
    return; // Restart the loop immediately
  }
  
  CalcError();
  
  PID_Turn(); // PID Control and Output to motors to turn
  RunMotors(); // Uses info from
  
  //Print(); // Print values to serial monitor
 
} // end loop()

// ************************************************************************************************* //
// NEW: SAFETY FUNCTION
// Returns TRUE if it is safe to drive.
// Returns FALSE if the robot is lifted, falling, or completely lost.
bool CheckSafety() {
  int darkCount = 0;
  int whiteCount = 0;

  // Scan all 7 sensors
  for (int i = 0; i < 7; i++) {
    // 1. Detect Darkness (Cliff / Lifted)
    // If sensor reads high (> 90), it sees no reflection (air/cliff) or black line.
    if (LDR[i] > 90) {
      darkCount++;
    }

    // 2. Detect White (Lost Line)
    // If sensor reads low (< 15), it sees only white surface.
    if (LDR[i] < 15) {
      whiteCount++;
    }
  }

  // CONDITION 1: CLIFF DETECTED
  // If 6 or more sensors are "Dark", we are likely in the air or off a cliff.
  if (darkCount >= 6) {
    // Serial.println("SAFETY TRIGGER: CLIFF / LIFTED");
    return false; // Unsafe
  }

  // CONDITION 2: LINE LOST
  // If ALL 7 sensors see white, we have lost the line completely.
  // Stop to prevent wandering into walls.
  if (whiteCount >= 7) {
    // Serial.println("SAFETY TRIGGER: LINE LOST");
    return false; // Unsafe
  }

  return true; // Safe
}

// ************************************************************************************************* //
// NEW: EMERGENCY STOP FUNCTION
void EmergencyStop() {
  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
  Motor1->setSpeed(0);
  Motor2->setSpeed(0);
}

// ************************************************************************************************* //
// function to quickly blink an LED
void fastBlink() {
  digitalWrite(led_Pin, HIGH); // turn on the LED
  delay(100); // wait for 100 milliseconds

  digitalWrite(led_Pin, LOW); // turn off the LED
  delay(100); // wait for 100 milliseconds
} // end fastBlink()

// ************************************************************************************************* //
// function to slowly blink an LED
void slowBlink() {
  digitalWrite(led_Pin, HIGH); // turn on the LED
  delay(100); // wait for 100 milliseconds

  digitalWrite(led_Pin, LOW); // turn off the LED
  delay(900); // wait for 900 milliseconds
} // end slowBlink()

// ************************************************************************************************* //
// function to calibrate
void Calibrate() { 
  // wait to make sure cart is in position with sensors over a white (light) surface
  for (int calii = 0; calii < 1; calii++) { // wait four blinks for the cart to be positioned
    slowBlink(); // blink the indicator LED
  }
  
  // Calibration
  // White Calibration
  for (int calii = 0; calii < numMeas; calii++) { // Repeat for numMeas (40)

    fastBlink(); // blink the indicator LED

    for (int ci = 0; ci < 7; ci++) { // loop over all 7 LDRs
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); 
      delay(2); 
    } 
  } 
    
  // Find average of each LDR's read values
  for (int cm = 0; cm < 7; cm++) { // loop over all 7 LDRs
    Mn[cm] = round(LDRf[cm] / (float)numMeas); // take average of each LDR's readings
    LDRf[cm] = 0.; // reset the array holding the sum of the LDR's readings
  }
 
 // Wait to move from White to Black Surface
  for (int calii = 0; calii < 2; calii++) { // wait ten blinks for the cart to be repositioned
    slowBlink();
  }
 
  // Black Calibration
  for (int calii = 0; calii < numMeas; calii++) { // Repeat for numMeas (40)

    fastBlink(); // blink the indicator LED

    for (int ci = 0; ci < 7; ci++) { // loop over all 7 LDRs
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); 
      delay(2); 
    } 
  }  
  
  // Find average of each LDR's read values
  for (int cm = 0; cm < 7; cm++) { // loop over all 7 LDRs
    Mx[cm] = round(LDRf[cm] / (float)numMeas); // take average of each LDR's readings
    LDRf[cm] = 0.; // reset the array holding the sum of the LDR's readings
  }
} // end Calibrate()

// ************************************************************************************************* //
// function to read and map values from potentiometers
void ReadPotentiometers() {
  SpRead = map(analogRead(S_pin), 0, 1023, 0, speed);
  kPRead = map(analogRead(P_pin), 0, 1023, 0, proportion);
  kIRead = map(analogRead(I_pin), 0, 1023, 0, integral);
  kDRead = map(analogRead(D_pin), 0, 1023, 0, derivative);
} // end ReadPotentiometers()
 
// ************************************************************************************************* //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() { 
  M1SpeedtoMotor = min(M1Sp + SpRead + M1P, 255); // limits speed to 255 
  M2SpeedtoMotor = min(M2Sp + SpRead + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  
  Motor1->setSpeed(abs(M1SpeedtoMotor)); 
  Motor2->setSpeed(abs(M2SpeedtoMotor));
  
  // Motor 1 control
  if (M1SpeedtoMotor > 0) {
    Motor1->run(FORWARD);
  } else { // < 0
    Motor1->run(BACKWARD);
  }

  // Motor 2 control
  if (M2SpeedtoMotor > 0) {
    Motor2->run(FORWARD);
  } else { // < 0 
    Motor2->run(BACKWARD);
  }
} // end RunMotors()

// ************************************************************************************************* //
// function to read photo resistors, map from 0 to 100, and find darkest photo resitor (MxIndex)
void ReadPhotoResistors() {
  for (int Li = 0; Li < 7; Li++) { // loop over all 7 LDRs
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    //delay(2); 
  }
} // end ReadPhotoResistors()

// ************************************************************************************************* //
// Calculate error from photoresistor readings
//  Do not worry about understanding or completing this function
void CalcError() {
  MxRead = -99; // initialize max read to an impossible value to ensure initialization does not impact functionality
  AveRead = 0.0; // initialize the average read

  for (int i = 0; i < 7; i = i + 1) { // loop over all of the LDRs
    if (MxRead < LDR[i]) { // if LDR value is greater than current max
      MxRead = LDR[i]; // set max equal to LDR value
      MxIndex = -1 * (i-3);
      im1 = (float)i;
    }
    AveRead = AveRead + (float)LDR[i]/7.;
  } 
  CriteriaForMax = 2; // max should be at least twice as big as the other values 
  if (MxRead > CriteriaForMax*AveRead) {
    if (im1 != 0 && im1 != 6) {
      im0 = im1 - 1;
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im0] * im0 + LDR[im1]*im1 + LDR[im2] * im2))/((float)(LDR[im0] + LDR[im1] + LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 0) {
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im1]*im1 + LDR[im2]*im2))/((float)(LDR[im1]+LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 6) {
      im0 = im1 - 1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1))/((float)(LDR[im0]+LDR[im1]));
      error = -1 * (WeightedAve - 3);
    } 
  } 

  if (isnan(error)) {
    error = lasterror;
  } else {
    error = error;
  }
} // end CalcError()

// ************************************************************************************************* //
// function to make a turn (a basic P controller)
void PID_Turn() {
  // Read values are scaled PID constants from potentiometers
  kP = (float)kPRead; 
  kI = (float)kIRead; 
  kD = (float)kDRead;

  // error holds values from -3 to 3
  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; //PID!!!!!
  
  sumerror = sumerror + error;

  // prevents integrator wind-up
  if (sumerror > 5) {
    sumerror = 5; 
  } else if (sumerror < -5) {
    sumerror = -5;
  }
  
  lasterror = error;
  
  // One motor becomes slower and the other faster, to "turn"
  if (Turn < 0) { // turn in direction of M2
    M1P = Turn; 
    M2P = -Turn;
  } else if (Turn > 0) { // turn in direction of M1
    M1P = Turn; 
    M2P = -Turn;
  } else { // continue in straight line
    M1P = 0; 
    M2P = 0;
  } 
} // end PID_Turn()

// ************************************************************************************************* //
// function to print values of interest
void Print() {
  Serial.print(Turn); Serial.print(" ");
  Serial.print(error); Serial.print(" ");
  Serial.print(sumerror); Serial.print(" ");
  Serial.print(lasterror); Serial.print(" ");
  
  Serial.print("   |   "); // create a visual divider
  
  Serial.print(SpRead); Serial.print(" "); // Initial Speed addition from potentiometer
  Serial.print(kP); Serial.print(" "); // PID values from potentiometers after scaling 
  Serial.print(kI); Serial.print(" ");
  Serial.print(kD); Serial.print(" ");

  Serial.print("   |   "); // create a visual divider
  
  Serial.print(LDR[0]); Serial.print(" "); // Each photo resistor value is shown
  Serial.print(LDR[1]); Serial.print(" ");
  Serial.print(LDR[2]); Serial.print(" ");
  Serial.print(LDR[3]); Serial.print(" ");
  Serial.print(LDR[4]); Serial.print(" ");
  Serial.print(LDR[5]); Serial.print(" ");
  Serial.print(LDR[6]); Serial.print(" "); 

  Serial.print("   |   "); // create a visual divider
  
  Serial.print(MxRead); Serial.print(" "); // the maximum value from the photo resistors is shown again
  Serial.print(MxIndex);Serial.print(" "); // this is the index of that maximum (0 through 6) (aka which element in LDR)
  Serial.print(error); Serial.print(" "); // this will show the calculated error (-3 through 3) 

  Serial.print("   |   "); // create a visual divider
  
  Serial.print(M1SpeedtoMotor); Serial.print(" "); // This prints the arduino output to each motor so you can see what the values (0-255)
  Serial.println(M2SpeedtoMotor); // that are sent to the motors would be without actually needing to power/run the motors
  
  delay(200); //just here to slow down the output for easier reading if wanted 
                  // ensure delay is commented when actually running your robot or this will slow down sampling too much
                  // and prevent the cart from functioning well
}