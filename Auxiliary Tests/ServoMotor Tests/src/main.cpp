#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// Define stepper motor connections and interface type
#define dirPin 4
#define stepPin 2
#define motorInterfaceType 1

#define armServoPin 27
#define turnServoPin 33

// Create an instance of the AccelStepper class
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
Servo armServo;
Servo turnServo;

// Define initial speed and delay parameters for smooth servo control
int minDelay = 30;      // Minimum delay in milliseconds
int maxDelay = 50;     // Maximum delay in milliseconds
int currentArmServoPos = 30; // Start from the middle position
int currentTurnServoPos = 100;

// Revised cubic easing function with slower easing-in
float easeInOutCubicSlow(float t) {
  t *= 2;
  if (t < 1) {
    return 0.5 * (t * t * t);
  }
  t -= 2;
  return 0.5 * (t * t * t + 2);
}

// Smooth servo control function with modified easing
void smoothServoControl(int endPos, int servoNo) {
  int startPos;
  switch (servoNo)
  {
  case 1:
    startPos = currentArmServoPos;
    break;
  case 2:
    startPos = currentTurnServoPos;
    break;
  
  default:
    break;
  }
  int range = abs(endPos - startPos);
  int increment = (startPos < endPos) ? 1 : -1;

  for (int i = 0; i <= range; i++) {
    float progress = (float)i / range;
    float easedProgress = easeInOutCubicSlow(progress);
    int pos = startPos + increment * (easedProgress * range);
    switch (servoNo)
    {
    case 1:
      armServo.write(pos);
      break;
    case 2:
      turnServo.write(pos);
      break;
    default:
      break;
    }
    
    // Calculate delay based on eased progress
    int currentDelay = minDelay + (int)(easedProgress * (maxDelay - minDelay));
    delay(currentDelay);
  }
  
  switch (servoNo)
  {
  case 1:
    currentArmServoPos = endPos; // Update the current servo position
    break;
  case 2:
    currentTurnServoPos = endPos;
    break;
  
  default:
    break;
  }
  
}

void setup() {
  Serial.begin(9600);
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(500);       // Set maximum speed in steps per second
  stepper.setAcceleration(500);   // Set acceleration in steps per second^2

  // Attach the servo to the specified pin with pulse widths for MG996R
  armServo.attach(armServoPin, 500, 2500); // Min pulse width, Max pulse width
  turnServo.attach(turnServoPin, 500, 2500);

  // Initialize servo to a known position
  armServo.write(currentArmServoPos); // Set to up
  turnServo.write(currentTurnServoPos); //set to forward
  delay(100); // Wait to ensure the servo reaches the position
  smoothServoControl(60,1);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming integer value
    int inputValue = Serial.parseInt();
    Serial.print("You entered: ");
    Serial.println(inputValue);
    if(inputValue >= 180){
      inputValue -= 180;
      smoothServoControl(inputValue, 2);
    }
    else{
      smoothServoControl(inputValue, 1);
    }
    

    // Print the received value
    
  }
}
