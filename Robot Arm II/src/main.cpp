#include <Arduino.h>
#include <Servo.h>

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

const int JOY_X = A1;
const int JOY_Y = A0;
const int JOY_BTN = 2;

const int DEADZONE = 50;        
const int MOVE_SPEED = 1;       
const int UPDATE_DELAY = 30;   
const int MIN_ANGLE = 0;       
const int MAX_ANGLE = 180;     

bool joyBtnLastState = HIGH;  

void setup() {
  baseServo.attach(BASE_SERVO_PIN);
  armServo.attach(ARM_SERVO_PIN);
  pinMode(JOY_BTN, INPUT_PULLUP);
  baseServo.write(basePos);
  armServo.write(armPos);
  delay(500);
}

void loop() {
  int joyX = analogRead(JOY_X);
  int joyY = analogRead(JOY_Y);
  
  bool joyBtn = digitalRead(JOY_BTN);
  controlServo(baseServo, basePos, joyX, 512, "BASE");  
  controlServo(armServo, armPos, joyY, 512, "ARM"); 
  
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

void resetRobotArm() {
  smoothMove(baseServo, basePos, 90);
  basePos = 90;
  smoothMove(armServo, armPos, 90);
  armPos = 90;
}


void smoothMove(Servo &servo, int currentPos, int targetPos) {
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(20);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(20);
    }
  }
}

void emergencyStop() {
  smoothMove(baseServo, basePos, 90);
  smoothMove(armServo, armPos, 90);
  basePos = 90;
  armPos = 90;
}