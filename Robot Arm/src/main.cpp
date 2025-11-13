#include <Arduino.h>
/*
 * Joystick Controlled Robot Arm
 * Control 3 servo motors using 2 analog joysticks
 * 
 * Hardware:
 * - Arduino Uno
 * - 3x Servo Motors
 * - 2x Analog Joystick Modules
 * - External Power Supply (recommended for servos)
 * 
 * Connections:
 * SERVOS:
 * - Servo 1 (Base) -> Pin 9
 * - Servo 2 (Shoulder) -> Pin 10
 * - Servo 3 (Elbow/Gripper) -> Pin 11
 * 
 * JOYSTICK 1 (Left):
 * - VRx (X-axis) -> A0
 * - VRy (Y-axis) -> A1
 * - SW (Button) -> Pin 2
 * 
 * JOYSTICK 2 (Right):
 * - VRx (X-axis) -> A2
 * - VRy (Y-axis) -> A3
 * - SW (Button) -> Pin 3
 */

#include <Servo.h>

// Create servo objects
Servo servo1;  // Base rotation
Servo servo2;  // Shoulder
Servo servo3;  // Elbow or Gripper

// Servo pins
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;

// Joystick 1 pins (Left joystick)
const int JOY1_X = A0;
const int JOY1_Y = A1;
const int JOY1_BTN = 2;

// Joystick 2 pins (Right joystick)
const int JOY2_X = A2;
const int JOY2_Y = A3;
const int JOY2_BTN = 3;

// Servo positions (0-180 degrees)
int servo1Pos = 90;
int servo2Pos = 90;
int servo3Pos = 90;

// Joystick center values (calibrated)
int joy1XCenter = 512;
int joy1YCenter = 512;
int joy2XCenter = 512;
int joy2YCenter = 512;

// Deadzone for joystick (prevents jitter)
const int DEADZONE = 50;

// Speed control
const int MOVE_SPEED = 2;  // Degrees per update
const int UPDATE_DELAY = 20; // Milliseconds between updates

// Button states
bool joy1BtnPressed = false;
bool joy2BtnPressed = false;
bool joy1BtnLastState = HIGH;
bool joy2BtnLastState = HIGH;

// Forward declarations (required in .cpp to avoid 'not declared in this scope' errors)
void calibrateJoysticks();
void resetToHome();
void printPositions();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Attach servos to pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  
  // Set up button pins
  pinMode(JOY1_BTN, INPUT_PULLUP);
  pinMode(JOY2_BTN, INPUT_PULLUP);
  
  // Move servos to initial position
  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
  servo3.write(servo3Pos);
  
  // Wait for servos to reach position
  delay(500);
  
  // Calibrate joystick center positions
  calibrateJoysticks();
  
  // Print startup message
  Serial.println("====================================");
  Serial.println("  Joystick Robot Arm Controller");
  Serial.println("====================================");
  Serial.println("LEFT JOYSTICK:");
  Serial.println("  X-axis: Control Servo 1 (Base)");
  Serial.println("  Y-axis: Control Servo 2 (Shoulder)");
  Serial.println("  Button: Reset to home position");
  Serial.println();
  Serial.println("RIGHT JOYSTICK:");
  Serial.println("  Y-axis: Control Servo 3 (Elbow/Gripper)");
  Serial.println("  Button: Print current positions");
  Serial.println("====================================");
  Serial.println();
}

void loop() {
  // Read joystick values
  int joy1X = analogRead(JOY1_X);
  int joy1Y = analogRead(JOY1_Y);
  int joy2Y = analogRead(JOY2_Y);
  
  // Read button states
  bool joy1BtnState = digitalRead(JOY1_BTN);
  bool joy2BtnState = digitalRead(JOY2_BTN);
  
  // Handle Joystick 1 X-axis -> Servo 1 (Base rotation)
  int joy1XDiff = joy1X - joy1XCenter;
  if (abs(joy1XDiff) > DEADZONE) {
    if (joy1XDiff > 0) {
      servo1Pos = constrain(servo1Pos + MOVE_SPEED, 0, 180);
    } else {
      servo1Pos = constrain(servo1Pos - MOVE_SPEED, 0, 180);
    }
    servo1.write(servo1Pos);
  }
  
  // Handle Joystick 1 Y-axis -> Servo 2 (Shoulder)
  int joy1YDiff = joy1Y - joy1YCenter;
  if (abs(joy1YDiff) > DEADZONE) {
    if (joy1YDiff > 0) {
      servo2Pos = constrain(servo2Pos + MOVE_SPEED, 0, 180);
    } else {
      servo2Pos = constrain(servo2Pos - MOVE_SPEED, 0, 180);
    }
    servo2.write(servo2Pos);
  }
  
  // Handle Joystick 2 Y-axis -> Servo 3 (Elbow/Gripper)
  int joy2YDiff = joy2Y - joy2YCenter;
  if (abs(joy2YDiff) > DEADZONE) {
    if (joy2YDiff > 0) {
      servo3Pos = constrain(servo3Pos + MOVE_SPEED, 0, 180);
    } else {
      servo3Pos = constrain(servo3Pos - MOVE_SPEED, 0, 180);
    }
    servo3.write(servo3Pos);
  }
  
  // Handle Joystick 1 Button (Reset to home)
  if (joy1BtnState == LOW && joy1BtnLastState == HIGH) {
    delay(50); // Debounce
    if (digitalRead(JOY1_BTN) == LOW) {
      resetToHome();
    }
  }
  joy1BtnLastState = joy1BtnState;
  
  // Handle Joystick 2 Button (Print positions)
  if (joy2BtnState == LOW && joy2BtnLastState == HIGH) {
    delay(50); // Debounce
    if (digitalRead(JOY2_BTN) == LOW) {
      printPositions();
    }
  }
  joy2BtnLastState = joy2BtnState;
  
  // Small delay for smooth operation
  delay(UPDATE_DELAY);
}

/**
 * Calibrate joystick center positions
 */
void calibrateJoysticks() {
  Serial.println("Calibrating joysticks...");
  Serial.println("Please keep joysticks centered!");
  delay(2000);
  
  // Read center values
  joy1XCenter = analogRead(JOY1_X);
  joy1YCenter = analogRead(JOY1_Y);
  joy2XCenter = analogRead(JOY2_X);
  joy2YCenter = analogRead(JOY2_Y);
  
  Serial.println("Calibration complete!");
  Serial.print("Joy1 Center: X=");
  Serial.print(joy1XCenter);
  Serial.print(" Y=");
  Serial.println(joy1YCenter);
  Serial.print("Joy2 Center: X=");
  Serial.print(joy2XCenter);
  Serial.print(" Y=");
  Serial.println(joy2YCenter);
  Serial.println();
}

/**
 * Reset all servos to home position (90 degrees)
 */
void resetToHome() {
  Serial.println("→ Resetting to home position...");
  
  // Smoothly move to home position
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

/**
 * Print current servo positions
 */
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