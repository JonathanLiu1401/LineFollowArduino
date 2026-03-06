/* ************************************************************************************************* */
// * ECE 201: Line Following Robot with PID                                                         *
/* ************************************************************************************************* */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // left motor
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2); // right motor

const int M1Sp = 60;
const int M2Sp = 60;

const int S_pin = A0;
const int P_pin = A1;
const int I_pin = A2;
const int D_pin = A3;

const float speed      = 140;
const float proportion = 120;
const float max_k_drop = 80;
const float derivative = 170;

float previous_derivative = 0.0;
float alpha = 0.4;

const int numMeas = 40;

int   SpRead      = 0;
int   kPRead      = 0;
float K_drop_read = 0;
int   kDRead      = 0;

int LDR_Pin[7] = {A8, A9, A10, A11, A12, A13, A14};
int LDR[7];
int led_Pin = 32;

float Mn[7];
float Mx[7];
float LDRf[7] = {0., 0., 0., 0., 0., 0., 0.};

int   MxRead;
int   MxIndex;
float AveRead;
int   CriteriaForMax;
float WeightedAve;

int im0, im1, im2;

int   M1SpeedtoMotor, M2SpeedtoMotor;
int   Turn;
int   M1P = 0, M2P = 0;
float error     = 0.0;
float lasterror = 0.0;
int   sumerror  = 0;
float kP, kI, kD;
int   counter   = 0;

unsigned long runStartTime = 0; // set after calibration — clock starts when car begins moving

// ************************************************************************************************* //
void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(led_Pin, OUTPUT);
  Calibrate();
  ReadPotentiometers();
  runStartTime = millis(); // start clock here, after calibration
  RunMotors();
}

// ************************************************************************************************* //
void loop() {
  ReadPhotoResistors();

  if (counter % 5000 == 0) {
    ReadPotentiometers();
  }
  counter++;

  if (CheckSafety() == false) {
    EmergencyStop();
    return;
  }

  CalcError();
  PID_Turn();
  RunMotors();

  // Print();
}

// ************************************************************************************************* //
bool CheckSafety() {
  int darkCount  = 0;
  int whiteCount = 0;
  for (int i = 0; i < 7; i++) {
    if (LDR[i] > 90) darkCount++;
    if (LDR[i] < 15) whiteCount++;
  }
  if (darkCount  >= 6) return false;
  if (whiteCount >= 7) return false;
  return true;
}

// ************************************************************************************************* //
void EmergencyStop() {
  Motor1->run(RELEASE); Motor2->run(RELEASE);
  Motor1->setSpeed(0);  Motor2->setSpeed(0);
}

// ************************************************************************************************* //
void Calibrate() {
  digitalWrite(led_Pin, HIGH);
  for (int calii = 0; calii < numMeas; calii++) {
    for (int ci = 0; ci < 7; ci++) { LDRf[ci] += (float)analogRead(LDR_Pin[ci]); delay(2); }
  }
  digitalWrite(led_Pin, LOW);
  for (int cm = 0; cm < 7; cm++) { Mn[cm] = round(LDRf[cm] / (float)numMeas); LDRf[cm] = 0.; }

  delay(1000);

  digitalWrite(led_Pin, HIGH);
  for (int calii = 0; calii < numMeas; calii++) {
    for (int ci = 0; ci < 7; ci++) { LDRf[ci] += (float)analogRead(LDR_Pin[ci]); delay(2); }
  }
  digitalWrite(led_Pin, LOW);
  for (int cm = 0; cm < 7; cm++) { Mx[cm] = round(LDRf[cm] / (float)numMeas); LDRf[cm] = 0.; }
}

// ************************************************************************************************* //
void ReadPotentiometers() {
  SpRead      = map(analogRead(S_pin), 0, 1023, 0, speed);
  kPRead      = map(analogRead(P_pin), 0, 1023, 0, proportion);
  K_drop_read = map(analogRead(I_pin), 0, 1023, 0, max_k_drop);
  kDRead      = map(analogRead(D_pin), 0, 1023, 0, derivative);
}

// ************************************************************************************************* //
void RunMotors() {
  // After 15s, halve the potentiometer speed boost for the stacked turn section
  int effectiveSpRead = (millis() - runStartTime < 8500 || millis() - runStartTime >= 17500)
                      ? SpRead : SpRead / 2.3;

  int currentBaseSpeed1 = M1Sp + effectiveSpRead;
  int currentBaseSpeed2 = M2Sp + effectiveSpRead;

  M1SpeedtoMotor = constrain(currentBaseSpeed1 + M1P, -255, 255);
  M2SpeedtoMotor = constrain(currentBaseSpeed2 + M2P, -255, 255);

  Motor1->setSpeed(abs(M1SpeedtoMotor));
  Motor2->setSpeed(abs(M2SpeedtoMotor));
  Motor1->run(M1SpeedtoMotor > 0 ? FORWARD : BACKWARD);
  Motor2->run(M2SpeedtoMotor > 0 ? FORWARD : BACKWARD);
}

// ************************************************************************************************* //
void ReadPhotoResistors() {
  for (int Li = 0; Li < 7; Li++) {
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
  }
}

// ************************************************************************************************* //
void CalcError() {
  MxRead  = -99;
  AveRead = 0.0;

  for (int i = 0; i < 7; i++) {
    if (MxRead < LDR[i]) { MxRead = LDR[i]; MxIndex = -1 * (i - 3); im1 = i; }
    AveRead += (float)LDR[i] / 7.0;
  }

  CriteriaForMax = 2;
  if (MxRead > CriteriaForMax * AveRead) {
    if (im1 != 0 && im1 != 6) {
      im0         = im1 - 1;
      im2         = im1 + 1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1 + LDR[im2]*im2))
                  / ((float)(LDR[im0] + LDR[im1] + LDR[im2]));
      error       = -1.0 * (WeightedAve - 3.0);
    } else if (im1 == 0) {
      im2         = im1 + 1;
      WeightedAve = ((float)(LDR[im1]*im1 + LDR[im2]*im2))
                  / ((float)(LDR[im1] + LDR[im2]));
      error       = -1.0 * (WeightedAve - 3.0);
    } else {
      im0         = im1 - 1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1))
                  / ((float)(LDR[im0] + LDR[im1]));
      error       = -1.0 * (WeightedAve - 3.0);
    }
  } else {
    error = -1.0 * (im1 - 3.0);
  }

  if (isnan(error)) error = lasterror;
}

// ************************************************************************************************* //
void PID_Turn() {
  kP = (float)kPRead;
  kI = 0.0;
  kD = (float)kDRead;

  float raw_derivative      = error - lasterror;
  float filtered_derivative = (alpha * raw_derivative) + ((1.0 - alpha) * previous_derivative);
  previous_derivative       = filtered_derivative;

  float quadratic_P = kP * error * (abs(error) / 3.0);
  Turn = (int)(quadratic_P + (sumerror * kI) + (filtered_derivative * kD));

  sumerror += (int)error;
  if      (sumerror >  5) sumerror =  5;
  else if (sumerror < -5) sumerror = -5;

  lasterror = error;

  M1P =  Turn;
  M2P = -Turn;
}

// ************************************************************************************************* //
void Print() {
  Serial.print(Turn);           Serial.print(" ");
  Serial.print(error);          Serial.print(" ");
  Serial.print(sumerror);       Serial.print(" ");
  Serial.print(lasterror);      Serial.print(" ");
  Serial.print("   |   ");
  Serial.print(SpRead);         Serial.print(" ");
  Serial.print(kP);             Serial.print(" ");
  Serial.print(K_drop_read);    Serial.print(" ");
  Serial.print(kD);             Serial.print(" ");
  Serial.print("   |   ");
  for (int i = 0; i < 7; i++) { Serial.print(LDR[i]); Serial.print(" "); }
  Serial.print("   |   ");
  Serial.print(MxRead);         Serial.print(" ");
  Serial.print(MxIndex);        Serial.print(" ");
  Serial.print(error);          Serial.print(" ");
  Serial.print("   |   ");
  Serial.print(M1SpeedtoMotor); Serial.print(" ");
  Serial.println(M2SpeedtoMotor);
  delay(200);
}