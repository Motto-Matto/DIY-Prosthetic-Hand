#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Keypad.h>

#define SERVOMIN 0  // Minimum pulse length count (out of 4096)
#define SERVOMAX 500  // Maximum pulse length count (out of 4096)
#define START_POSITION 100  // Initial servo position (midway between SERVOMIN and SERVOMAX)
#define NUM_SERVOS 5  // Total number of servos
#define STEP_SIZE 10  // Smaller step size for smoother movement

// Setup for the PWM Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int p[NUM_SERVOS];  // Array to store PWM values for each servo
int currentMotor = 0;  // Index for the currently active motor

// Keypad Setup
const byte ROWS = 4;  // Four rows
const byte COLS = 4;  // Four columns
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};  // Connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2};  // Connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS );

void setup() {
  Serial.begin(9600);
  Serial.println("Servo Test - Press '1' to rotate forward, '2' to rotate backward, '0' to switch motor");

  pwm.begin();
  pwm.setPWMFreq(50);  // Set frequency to 50Hz for servo motors
  delay(10);

  // Initialize PWM values for all motors to the starting position
  for (int i = 0; i < NUM_SERVOS; i++) {
    p[i] = START_POSITION;
    pwm.setPWM(i, 0, p[i]);  // Set initial position for each motor
  }
  Serial.print("Initial PWM (Servo Position for all motors): ");
  Serial.println(START_POSITION);
}

void loop() {
  // Check for a keypress from the keypad
  char key = keypad.getKey();
  
  if (key) {
    // Print the key pressed to the Serial Monitor
    Serial.print("Key Pressed: ");
    Serial.println(key);

    if (key == '1') {  // Rotate active motor forward (increase PWM)
      if (p[currentMotor] + STEP_SIZE <= SERVOMAX) {
        p[currentMotor] += STEP_SIZE;  // Increase PWM value to rotate the servo
        pwm.setPWM(currentMotor, 0, p[currentMotor]);
        Serial.print("Motor ");
        Serial.print(currentMotor);
        Serial.print(" rotated forward, PWM: ");
        Serial.println(p[currentMotor]);
      }
    }
    else if (key == '2') {  // Rotate active motor backward (decrease PWM)
      if (p[currentMotor] - STEP_SIZE >= SERVOMIN) {
        p[currentMotor] -= STEP_SIZE;  // Decrease PWM value to rotate the servo
        pwm.setPWM(currentMotor, 0, p[currentMotor]);
        Serial.print("Motor ");
        Serial.print(currentMotor);
        Serial.print(" rotated backward, PWM: ");
        Serial.println(p[currentMotor]);
      }
    }
    else if (key == '0') {  // Switch to the next motor
      currentMotor++;  // Move to the next motor
      if (currentMotor >= NUM_SERVOS) {
        currentMotor = 0;  // If the motor exceeds the total, loop back to motor 0
      }
      Serial.print("Switched to motor ");
      Serial.println(currentMotor);
    }
  }
  
  delay(100);  // Small delay to prevent excessive reads from serial
}
