#include <Arduino.h>
#include "AccelStepper.h"
#include <ESP32Servo.h>

#define commBit0 21
#define commBit1 22
#define commBit2 19
#define commBit3 8
#define signal 7
#define ready 5


// Define stepper motor connections and interface type
#define dirPin 4
#define stepPin 2
#define motorInterfaceType 1

#define plateArmServoPin 26
#define fryArmServoPin 25
#define cutServoPin 33
#define botServoPin 32

// Create an instance of the AccelStepper class
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
Servo plateArmServo;
Servo fryArmServo;
Servo cutServo;
Servo botServo;

// Define initial speed and delay parameters for smooth servo control
int minDelay = 30;      // Minimum delay in milliseconds
int maxDelay = 50;     // Maximum delay in milliseconds
int currentPlateServoPos = 100; // Start from the middle position
int currentFryServoPos = 100;
int currentCutServoPos = 100;
int currentBotServoPos = 100;

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
/*
 *@brief controls a servo motor with cubic easing
 *
 * @param endPos the absolute end pos (0 <= pos <= 180) to move to
 * @param servoNo the servo to move
 */
void smoothServoControl(int endPos, int servoNo) {

  int startPos;
  switch (servoNo)
  {
  case 1:
    startPos = currentPlateServoPos;
    break;
  
  case 2:
    startPos = currentFryServoPos;
    break;
  
  case 3:
    startPos = currentCutServoPos;
    break;
  
  case 4:
    startPos = currentBotServoPos;
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
      plateArmServo.write(pos);
      break;
    
    case 2:
      fryArmServo.write(pos);
      break;
    
    case 3:
      cutServo.write(pos);
      break;
    
    case 4:
      botServo.write(pos);
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
      currentPlateServoPos = endPos;
      break;
    
    case 2:
      currentFryServoPos = endPos;
      break;
    
    case 3:
      currentCutServoPos = endPos;
      break;
    
    case 4:
      currentBotServoPos = endPos;
      break;
    
    default:
      break;
    }
}


void setCommPinOutput(int taskNumber);
int readCommPinInput();
void waitForSignal(int pin);
void performTask(int taskNo);

void setup() {
   Serial.begin(115200);
  pinMode(signal, INPUT);  // ESP-1 will read this pin
  pinMode(ready, OUTPUT);  // ESP-1 will control this pin
  setCommPinOutput(0);
  // Attach the servo to the specified pin with pulse widths for MG996R
  plateArmServo.attach(plateArmServoPin, 500, 2500); // Min pulse width, Max pulse width
  fryArmServo.attach(fryArmServoPin, 500, 2500);
  botServo.attach(botServoPin, 500, 2500);
  cutServo.attach(cutServoPin, 500, 2500);


  // Initialize servo to a known position
  plateArmServo.write(100);
  fryArmServo.write(100);
  botServo.write(100);
  cutServo.write(100);
  delay(100); // Wait to ensure the servo reaches the position
}

void loop(){
  /*
  if (Serial.available() > 0) {
    // Read the incoming integer value
    int inputValue = Serial.parseInt();
    Serial.print("You entered: ");
    Serial.println(inputValue);
    
    performTask(inputValue);
  }
  */
 performTask(1);
 delay(1000);
 performTask(2);
 vTaskDelete(NULL);
}

/* Function to perform a single task */

/*
 *@brief perform a predetermined task while communicating over a wire connection
 *
 * @param taskNo the task number to call
 */
void performTask(int taskNo) {
  Serial.println("Performing Tasks");
  int angle;
  int servoNo;
  switch (taskNo)
  {
  case 1:
    angle = 0;
    servoNo = 1;
    break;
  
  case 2:
    angle = 180;
    servoNo = 1;
    break;

  default:
    break;
  }

  setCommPinOutput(taskNo);
  Serial.print("Task Number Sent: ");
  Serial.println(taskNo);

  digitalWrite(ready, HIGH);  // Signal ESP-2 that data is ready
  smoothServoControl(angle, servoNo);
  waitForSignal(signal);  // Wait for ESP-2 to complete the task
  digitalWrite(ready, LOW);  // Reset the ready signal

  // After receiving the signal from ESP-2, reset the output
  setCommPinOutput(0);
  Serial.println("Received completion signal from ESP-2");
}

/* Helper function to wait for signal pin to go LOW */
void waitForSignal(int pin) {
  while (digitalRead(pin) == HIGH) {
    Serial.println("Waiting for signal to go LOW...");
    delay(10);
  }
  Serial.println("Signal is LOW, proceeding...");
}

/* @brief Converts integer into binary
 *
 * This method converts an integer value into a binary value to be sent down communication pins
 * 
 * @param integerValue the integer value to be converted
*/
void setCommPinOutput(int integerValue) {
  pinMode(commBit0, OUTPUT);
  pinMode(commBit1, OUTPUT);
  pinMode(commBit2, OUTPUT);
  pinMode(commBit3, OUTPUT);

  digitalWrite(commBit0, bitRead(integerValue, 0));
  digitalWrite(commBit1, bitRead(integerValue, 1));
  digitalWrite(commBit2, bitRead(integerValue, 2));
  digitalWrite(commBit3, bitRead(integerValue, 3));
}

/*
 * @brief Reads pin values and converts to integer
 *
 * The method reads the communication pins and converts the binary signal into an integer value
 * 
*/
int readCommPinInput() {
  pinMode(commBit0, INPUT);
  pinMode(commBit1, INPUT);
  pinMode(commBit2, INPUT);
  pinMode(commBit3, INPUT);

  int value = 0;
  value |= digitalRead(commBit0) << 0;
  value |= digitalRead(commBit1) << 1;
  value |= digitalRead(commBit2) << 2;
  value |= digitalRead(commBit3) << 3;

  return value;
}
