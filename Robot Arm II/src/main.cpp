#include <Arduino.h>
#include <Servo.h>
void calibrateJoysticks();

void controlServo(Servo &servo, int &currentPos, int joyValue, int joyCenter, String servoName);
void handleButtons(bool joyBtn);
void resetRobotArm();
void smoothMove(Servo &servo, int currentPos, int targetPos);
void emergencyStop();

Servo baseServo;   
Servo armServo;     

const int BASE_SERVO_PIN = 11;   
const int ARM_SERVO_PIN = 10;  
int basePos = 90;    
int armPos = 90;      

const int JOY_X = A0;
const int JOY_Y = A1;

const int JOY_BTN = 2;

int joyXCenter = 512;
int joyYCenter = 512;

const int DEADZONE = 50;        
const int MOVE_SPEED = 2;       
const int UPDATE_DELAY = 20;   
const int MIN_ANGLE = 0;       
const int MAX_ANGLE = 180;     

bool joyBtnLastState = HIGH;
bool controlMode = false; 
enum ControlMode {
  MODE_X_AXIS,    
  MODE_Y_AXIS,    
  MODE_BOTH      
};

ControlMode servo1Mode = MODE_Y_AXIS;  
ControlMode servo2Mode = MODE_Y_AXIS;  

void setup() {
  Serial.begin(9600);
  baseServo.attach(BASE_SERVO_PIN);
  armServo.attach(ARM_SERVO_PIN);
  pinMode(JOY_BTN, INPUT_PULLUP);
  baseServo.write(basePos);
  armServo.write(armPos);
  delay(500);
  calibrateJoysticks();
 
}

void loop() {
  int joyX = analogRead(JOY_X);
  int joyY = analogRead(JOY_Y);
  
  bool joyBtn = digitalRead(JOY_BTN);
  controlServo(baseServo, basePos, joyX, joyXCenter, "BASE");  
  controlServo(armServo, armPos, joyY, joyYCenter, "ARM"); 
  
  handleButtons(joyBtn);

  delay(UPDATE_DELAY);
}


void controlServo(Servo &servo, int &currentPos, int joyValue, int joyCenter, String servoName) {
  int joyDiff = joyValue - joyCenter;
  
  if (abs(joyDiff) > DEADZONE) {
    int newPos = currentPos;
    
    if (joyDiff > 0) {
      newPos = currentPos + MOVE_SPEED;
    } else {
      newPos = currentPos - MOVE_SPEED;
    }

    newPos = constrain(newPos, MIN_ANGLE, MAX_ANGLE);
    
    if (newPos != currentPos) {
      currentPos = newPos;
      servo.write(currentPos);
      
      // Optional: Print position updates
      // Serial.print(servoName);
      // Serial.print(" Joint: ");
      // Serial.print(currentPos);
      // Serial.println("°");
    }
  }
}


void handleButtons(bool joyBtn) {
  
  if (joyBtn == LOW && joyBtnLastState == HIGH) {
    delay(50); 
    if (digitalRead(JOY_BTN) == LOW) {
      resetRobotArm();
    }
  }
  joyBtnLastState = joyBtn;
}


void calibrateJoysticks() {
  delay(2000);
  
  long sumX = 0, sumY = 0;
  int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(10);
  }
  
  joyXCenter = sumX / samples;
  joyYCenter = sumY / samples;
  
}


void resetRobotArm() {
  smoothMove(baseServo, basePos, 90);
  basePos = 90;
  smoothMove(armServo, armPos, 90);
  armPos = 90;
  Serial.println();
}


void smoothMove(Servo &servo, int currentPos, int targetPos) {
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(15);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(15);
    }
  }
}





void emergencyStop() {
  smoothMove(baseServo, basePos, 90);
  smoothMove(armServo, armPos, 90);
  basePos = 90;
  armPos = 90;
}