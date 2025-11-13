#include <Arduino.h>
#include <Servo.h>

Servo servo1;  
Servo servo2;  
Servo servo3; 


const int SERVO1_PIN = 11;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 6;

const int JOY1_X = A0;
const int JOY1_Y = A1;
const int JOY1_BTN = 2;

const int JOY2_X = A2;
const int JOY2_Y = A3;
const int JOY2_BTN = 3;

int servo1Pos = 90;
int servo2Pos = 90;
int servo3Pos = 90;

int joy1XCenter = 512;
int joy1YCenter = 512;
int joy2XCenter = 512;
int joy2YCenter = 512;

const int DEADZONE = 50;

const int MOVE_SPEED = 2;  
const int UPDATE_DELAY = 20; 

bool joy1BtnPressed = false;
bool joy2BtnPressed = false;
bool joy1BtnLastState = HIGH;
bool joy2BtnLastState = HIGH;

void calibrateJoysticks();
void resetToHome();
void printPositions();

void setup() {
  Serial.begin(9600);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  pinMode(JOY1_BTN, INPUT_PULLUP);
  pinMode(JOY2_BTN, INPUT_PULLUP);
  
  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
  servo3.write(servo3Pos);
  
  delay(500);
  
  calibrateJoysticks();
  
}

void loop() {
  int joy1X = analogRead(JOY1_X);
  int joy1Y = analogRead(JOY1_Y);
  int joy2Y = analogRead(JOY2_Y);
  
  bool joy1BtnState = digitalRead(JOY1_BTN);
  bool joy2BtnState = digitalRead(JOY2_BTN);
  
  int joy1XDiff = joy1X - joy1XCenter;
  if (abs(joy1XDiff) > DEADZONE) {
    if (joy1XDiff > 0) {
      servo1Pos = constrain(servo1Pos + MOVE_SPEED, 0, 180);
    } else {
      servo1Pos = constrain(servo1Pos - MOVE_SPEED, 0, 180);
    }
    servo1.write(servo1Pos);
  }
  

  int joy1YDiff = joy1Y - joy1YCenter;
  if (abs(joy1YDiff) > DEADZONE) {
    if (joy1YDiff > 0) {
      servo2Pos = constrain(servo2Pos + MOVE_SPEED, 0, 180);
    } else {
      servo2Pos = constrain(servo2Pos - MOVE_SPEED, 0, 180);
    }
    servo2.write(servo2Pos);
  }
  

  int joy2YDiff = joy2Y - joy2YCenter;
  if (abs(joy2YDiff) > DEADZONE) {
    if (joy2YDiff > 0) {
      servo3Pos = constrain(servo3Pos + MOVE_SPEED, 0, 180);
    } else {
      servo3Pos = constrain(servo3Pos - MOVE_SPEED, 0, 180);
    }
    servo3.write(servo3Pos);
  }
  
 
  if (joy1BtnState == LOW && joy1BtnLastState == HIGH) {
    delay(50); 
    if (digitalRead(JOY1_BTN) == LOW) {
      resetToHome();
    }
  }
  joy1BtnLastState = joy1BtnState;
  
  
  if (joy2BtnState == LOW && joy2BtnLastState == HIGH) {
    delay(50); 
    if (digitalRead(JOY2_BTN) == LOW) {
      printPositions();
    }
  }
  joy2BtnLastState = joy2BtnState;
  
  
  delay(UPDATE_DELAY);
}


void calibrateJoysticks() {
  delay(2000);

  joy1XCenter = analogRead(JOY1_X);
  joy1YCenter = analogRead(JOY1_Y);
  joy2XCenter = analogRead(JOY2_X);
  joy2YCenter = analogRead(JOY2_Y);
  
}


void resetToHome() {
  while (servo1Pos != 90 || servo2Pos != 90 || servo3Pos != 90) {
    if (servo1Pos < 90) servo1Pos++;
    else if (servo1Pos > 90) servo1Pos--;
    
    if (servo2Pos < 90) servo2Pos++;
    else if (servo2Pos > 90) servo2Pos--;
    
    if (servo3Pos < 90) servo3Pos++;
    else if (servo3Pos > 90) servo3Pos--;
    
    servo1.write(servo1Pos);
    servo2.write(servo2Pos);
    servo3.write(servo3Pos);
    
    delay(15);
  }
  
  Serial.println("✓ Home position reached!");
}


void printPositions() {
  Serial.println("====================================");
  Serial.println("  CURRENT SERVO POSITIONS");
  Serial.println("====================================");
  Serial.print("  Servo 1 (Base):     ");
  Serial.print(servo1Pos);
  Serial.println("°");
  Serial.print("  Servo 2 (Shoulder): ");
  Serial.print(servo2Pos);
  Serial.println("°");
  Serial.print("  Servo 3 (Elbow):    ");
  Serial.print(servo3Pos);
  Serial.println("°");
  Serial.println("====================================");
  Serial.println();
}

/**
 * Optional: Advanced control with acceleration
 * Uncomment and modify loop() to use this function
 */
/*
void advancedControl() {
  // Read joystick values
  int joy1X = analogRead(JOY1_X);
  int joy1Y = analogRead(JOY1_Y);
  int joy2Y = analogRead(JOY2_Y);
  
  // Calculate speed based on joystick deflection
  int speed1 = map(abs(joy1X - joy1XCenter), DEADZONE, 512, 1, 5);
  int speed2 = map(abs(joy1Y - joy1YCenter), DEADZONE, 512, 1, 5);
  int speed3 = map(abs(joy2Y - joy2YCenter), DEADZONE, 512, 1, 5);
  
  // Apply speed control
  if (abs(joy1X - joy1XCenter) > DEADZONE) {
    servo1Pos = constrain(servo1Pos + ((joy1X > joy1XCenter) ? speed1 : -speed1), 0, 180);
    servo1.write(servo1Pos);
  }
  
  if (abs(joy1Y - joy1YCenter) > DEADZONE) {
    servo2Pos = constrain(servo2Pos + ((joy1Y > joy1YCenter) ? speed2 : -speed2), 0, 180);
    servo2.write(servo2Pos);
  }
  
  if (abs(joy2Y - joy2YCenter) > DEADZONE) {
    servo3Pos = constrain(servo3Pos + ((joy2Y > joy2YCenter) ? speed3 : -speed3), 0, 180);
    servo3.write(servo3Pos);
  }
}
*/