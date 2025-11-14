#include <Arduino.h>
#include <Servo.h>

void controlServo(Servo &servo, int &currentPos, int joyValue, int centerValue, float speed);
void handleButtons(bool joyBtn);
void resetRobotArm();
void smoothMove(Servo &servo, int currentPos, int targetPos);
void emergencyStop();

Servo baseServo;
Servo armServo;

const int BASE_SERVO_PIN = 11;
const int ARM_SERVO_PIN = 10;

int basePos = 90;
int armPos  = 90;

const int JOY_X = A1;
const int JOY_Y = A0;
const int JOY_BTN = 2;

const int DEADZONE_X = 40;   
const int DEADZONE_Y = 40;   

const float BASE_SPEED = 3.0;
const float ARM_SPEED  = 2.0;

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
  controlServo(baseServo, basePos, joyX, 512, BASE_SPEED);
  controlServo(armServo, armPos, joyY, 512, ARM_SPEED);

  handleButtons(joyBtn);
  delay(20);
}

void controlServo(Servo &servo, int &currentPos, int joyValue, int centerValue, float speed) {

  int diff = joyValue - centerValue;

  int deadzone = (servo.attached() && (&servo == &baseServo)) ? DEADZONE_X : DEADZONE_Y;

  if (abs(diff) > deadzone) {

    int newPos = currentPos;

    if (diff > 0) newPos += speed;
    else newPos -= speed;

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
      delay(10);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(10);
    }
  }
}

void emergencyStop() {
  smoothMove(baseServo, basePos, 90);
  smoothMove(armServo, armPos, 90);
  basePos = 90;
  armPos = 90;
}
