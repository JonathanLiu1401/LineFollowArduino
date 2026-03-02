/* ************************************************************************************************* */
// * ECE 201: Line Following Robot with PID + Dynamic Throttle & Safety Logic * //
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
const int S_pin = A0; // speed control
const int P_pin = A1; // proportional control
const int I_pin = A2; // dynamic deceleration (K_drop) control
const int D_pin = A3; // derivative control

// User set SPID values
// These are the MAXIMUM values that the potentiometers can map to.
const float speed       = 140; // Max added speed (Control Margin of 55 preserved)
const float proportion  = 80;  // Max Kp
const float max_k_drop  = 80;  // Max K_drop (Replaces Integral max)
const float derivative  = 100;  // Max Kd

// EMA Filter Variables for High-Speed Damping
float previous_derivative = 0.0; // Stores the previous filtered derivative
float alpha = 0.3;               // EMA smoothing factor (0.0 to 1.0)

// Statistical Sampling
const int numMeas = 50; // 30 samples executes in ~2.8 seconds with optimized logic

// Initialize SPID values to zero
int SpRead = 0; // Speed Increase 
int kPRead = 0; // Proportional gain
float K_drop_read = 0; // Dynamic deceleration factor
int kDRead = 0; // Derivative gain
  
// Variables for Light Sensors
// LDR_Pin holds 7 values of the LDR pin connection to the Arduino
int LDR_Pin[7] = {A8, A9, A10, A11, A12, A13, A14}; 
int LDR[7]; // Array to hold the mapped readings from photoresistors

// LED Pin (PRESET DO NOT CHANGE)
int led_Pin = 32; 

// Calibration Variables
float Mn[7]; // array containing minimum read values
float Mx[7]; // array containing maximum read values
float LDRf[7] = {0.,0.,0.,0.,0.,0.,0.}; // initialize the LDR read values to float zero

int MxRead; // maximum read value
int MxIndex; // index of max read value
float AveRead; // average read value
int CriteriaForMax; // value to determine whether a read value is a max
float WeightedAve; // weighted average
  
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
  
  ReadPotentiometers(); // Read potentiometer values
  
  delay(2000); // Delay to ensure cart is properly placed before moving
  
  RunMotors(); // Starts motors straight forward
 
} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {
  ReadPotentiometers(); // Read knobs to adjust tuning dynamically
  
  ReadPhotoResistors(); // Read photoresistors and map to 0-100 based on calibration
  
  // *** SAFETY CHECK ***
  // If CheckSafety returns false, danger is detected.
  // We stop the motors and skip the PID calculation.
  if (CheckSafety() == false) {
    EmergencyStop();
    return; // Restart the loop immediately
  }
  
  CalcError();
  
  PID_Turn(); // PID Control and Output to motors to turn
  RunMotors(); 
  
  // Print(); // Uncomment this line to tune on the track, re-comment for competition runs
 
} // end loop()

// ************************************************************************************************* //
// SAFETY FUNCTION
bool CheckSafety() {
  int darkCount = 0;
  int whiteCount = 0;

  for (int i = 0; i < 7; i++) {
    if (LDR[i] > 90) {
      darkCount++;
    }
    if (LDR[i] < 15) {
      whiteCount++;
    }
  }

  // CONDITION 1: CLIFF DETECTED
  if (darkCount >= 6) {
    return false; // Unsafe
  }

  // CONDITION 2: LINE LOST
  if (whiteCount >= 7) {
    return false; // Unsafe
  }

  return true; // Safe
}

// ************************************************************************************************* //
// EMERGENCY STOP FUNCTION
void EmergencyStop() {
  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
  Motor1->setSpeed(0);
  Motor2->setSpeed(0);
}

// ************************************************************************************************* //
// function to quickly blink an LED
void fastBlink() {
  digitalWrite(led_Pin, HIGH); 
  delay(100); 

  digitalWrite(led_Pin, LOW); 
  delay(100); 
} 

// ************************************************************************************************* //
// function to slowly blink an LED
void slowBlink() {
  digitalWrite(led_Pin, HIGH); 
  delay(100); 

  digitalWrite(led_Pin, LOW); 
  delay(900); 
} 

// ************************************************************************************************* //
// Optimized function to calibrate
void Calibrate() { 
  // 1. White Calibration (Assumes cart is powered ON while placed over white surface)
  digitalWrite(led_Pin, HIGH); // Turn LED ON to indicate active reading
  for (int calii = 0; calii < numMeas; calii++) { 
    for (int ci = 0; ci < 7; ci++) { 
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); 
      delay(2); 
    } 
  } 
  digitalWrite(led_Pin, LOW); // Turn LED OFF to indicate reading is done
    
  // 2. Find average of each LDR's white read values
  for (int cm = 0; cm < 7; cm++) { 
    Mn[cm] = round(LDRf[cm] / (float)numMeas); 
    LDRf[cm] = 0.; 
  }
 
  // 3. Wait to move from White to Black Surface
  // slowBlink(); // 1000 ms delay
  // slowBlink(); // 1000 ms delay
  delay(1000);
 
  // 4. Black Calibration (Fast, Continuous Reading)
  digitalWrite(led_Pin, HIGH); // Turn LED ON to indicate active reading
  for (int calii = 0; calii < numMeas; calii++) { 
    for (int ci = 0; ci < 7; ci++) { 
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); 
      delay(2); 
    } 
  }  
  digitalWrite(led_Pin, LOW); // Turn LED OFF
  
  // 5. Find average of each LDR's black read values
  for (int cm = 0; cm < 7; cm++) { 
    Mx[cm] = round(LDRf[cm] / (float)numMeas); 
    LDRf[cm] = 0.; 
  }
} // end Calibrate()

// ************************************************************************************************* //
// function to read and map values from potentiometers
void ReadPotentiometers() {
  SpRead = map(analogRead(S_pin), 0, 1023, 0, speed);
  kPRead = map(analogRead(P_pin), 0, 1023, 0, proportion);
  
  // Reassign I_pin to control the dynamic deceleration factor
  K_drop_read = map(analogRead(I_pin), 0, 1023, 0, max_k_drop);
  
  kDRead = map(analogRead(D_pin), 0, 1023, 0, derivative);
} // end ReadPotentiometers()
 
// ************************************************************************************************* //
// Optimized function to start motors with dynamic speed scaling
void RunMotors() { 
  // 1. Define the deceleration factor using the mapped potentiometer value
  float K_drop = K_drop_read; 
  
  // 2. Calculate the speed penalty based on the absolute error
  int speedPenalty = round(abs(error) * K_drop); 
  
  // 3. Calculate dynamic base speed
  int currentBaseSpeed1 = max((M1Sp + SpRead) - speedPenalty, 0); 
  int currentBaseSpeed2 = max((M2Sp + SpRead) - speedPenalty, 0); 
  
  // 4. Apply the PID turn calculations to the dynamic base speed
  M1SpeedtoMotor = currentBaseSpeed1 + M1P; 
  M2SpeedtoMotor = currentBaseSpeed2 + M2P; 
  
  // 5. Constrain absolute bounds to prevent 8-bit variable overflow
  M1SpeedtoMotor = constrain(M1SpeedtoMotor, -255, 255);
  M2SpeedtoMotor = constrain(M2SpeedtoMotor, -255, 255);
  
  // 6. Send absolute speeds to the Adafruit motor shield
  Motor1->setSpeed(abs(M1SpeedtoMotor)); 
  Motor2->setSpeed(abs(M2SpeedtoMotor));
  
  // 7. Motor 1 direction control
  if (M1SpeedtoMotor > 0) {
    Motor1->run(FORWARD);
  } else { 
    Motor1->run(BACKWARD);
  }

  // 8. Motor 2 direction control
  if (M2SpeedtoMotor > 0) {
    Motor2->run(FORWARD);
  } else {  
    Motor2->run(BACKWARD);
  }
} // end RunMotors()

// ************************************************************************************************* //
// function to read photo resistors (Optimized frequency)
void ReadPhotoResistors() {
  for (int Li = 0; Li < 7; Li++) { 
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
  }
} // end ReadPhotoResistors()

// ************************************************************************************************* //
// Calculate error from photoresistor readings
void CalcError() {
  MxRead = -99; 
  AveRead = 0.0; 

  for (int i = 0; i < 7; i = i + 1) { 
    if (MxRead < LDR[i]) { 
      MxRead = LDR[i]; 
      MxIndex = -1 * (i-3);
      im1 = (float)i;
    }
    AveRead = AveRead + (float)LDR[i]/7.;
  } 
  CriteriaForMax = 2; 
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
// function to make a turn (PID controller with EMA filter)
void PID_Turn() {
  // Read values are scaled PID constants from potentiometers
  kP = (float)kPRead; 
  kI = 0; // Hardcoded integral value
  kD = (float)kDRead;

  // 1. Calculate raw discrete derivative
  float raw_derivative = error - lasterror;

  // 2. Apply Exponential Moving Average (EMA) Filter
  float filtered_derivative = (alpha * raw_derivative) + ((1.0 - alpha) * previous_derivative);
  
  // 3. Store the filtered value for the next loop iteration
  previous_derivative = filtered_derivative;

  // 4. Calculate Turn output using the filtered derivative
  Turn = error * kP + sumerror * kI + filtered_derivative * kD;
  
  sumerror = sumerror + error;

  // prevents integrator wind-up
  if (sumerror > 5) {
    sumerror = 5; 
  } else if (sumerror < -5) {
    sumerror = -5;
  }
  
  lasterror = error;
  
  // One motor becomes slower and the other faster, to "turn"
  if (Turn < 0) { 
    M1P = Turn; 
    M2P = -Turn;
  } else if (Turn > 0) { 
    M1P = Turn; 
    M2P = -Turn;
  } else { 
    M1P = 0; 
    M2P = 0;
  } 
}

// ************************************************************************************************* //
// function to print values of interest
void Print() {
  Serial.print(Turn); Serial.print(" ");
  Serial.print(error); Serial.print(" ");
  Serial.print(sumerror); Serial.print(" ");
  Serial.print(lasterror); Serial.print(" ");
  
  Serial.print("   |   "); 
  
  Serial.print(SpRead); Serial.print(" "); 
  Serial.print(kP); Serial.print(" "); 
  Serial.print(K_drop_read); Serial.print(" "); // Prints your dynamic deceleration factor
  Serial.print(kD); Serial.print(" ");

  Serial.print("   |   "); 
  
  Serial.print(LDR[0]); Serial.print(" "); 
  Serial.print(LDR[1]); Serial.print(" ");
  Serial.print(LDR[2]); Serial.print(" ");
  Serial.print(LDR[3]); Serial.print(" ");
  Serial.print(LDR[4]); Serial.print(" ");
  Serial.print(LDR[5]); Serial.print(" ");
  Serial.print(LDR[6]); Serial.print(" "); 

  Serial.print("   |   "); 
  
  Serial.print(MxRead); Serial.print(" "); 
  Serial.print(MxIndex);Serial.print(" "); 
  Serial.print(error); Serial.print(" "); 

  Serial.print("   |   "); 
  
  Serial.print(M1SpeedtoMotor); Serial.print(" "); 
  Serial.println(M2SpeedtoMotor); 
  
  delay(200); 
}