#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define lineFrontLeft 39
#define lineFrontRight 36
#define lineBackLeft 33
#define lineBackRight 32

#define frontLeftSensor 34
#define backLeftSensor 35
#define frontRightSensor 37
#define backRightSensor 38

#define commBit0 21
#define commBit1 22
#define commBit2 19
#define commBit3 8
#define signal 7
#define readyPin 5

//Global state variables
bool doBurger = false;
bool ready = false;
bool move = true;
int state = 0;
int previousState = 0;
bool isThere = false;

//Global variables for counting lines
int currentLineCount = 0;
bool almostThere = false;

//booleans for sensor tripping on left and right sides
bool frontRightDetected = false;
bool frontLeftDetected = false;
bool backLeftDetected = false;
bool backRightDetected = false;
bool forwardDetected = false;
int isLeft = 0;

// PWM configuration
const int pwmFrequency = 300; // PWM frequency in Hz
const int pwmResolution = 12; // PWM resolution (1-16 bits)

//Motor pin declaration
const int motorL1 = 13;
const int motorL2 = 15;
const int motorR1 = 2;
const int motorR2 = 4;

//Assigning each motor lead individual PWM channels
const int L1 = 0;
const int L2 = 2;
const int R1 = 1;
const int R2 = 3;

//Constants for line detection
const int OPTICAL_SENSOR_THRESHOLD = 800;

//Navigation variables
int slowSpeed = 1350;
int regSpeed = 2539;
int centreTime = 200;
int firstStation = 1;
int stopSpeed = 4095;
int stopCount = 8;
int stopDelay = 10;

// PID control variables
float Kp = 0;
float Ki = 0;
float Kd = 0;
float previousError = 0;
float integral = 0;


void executeTask(void *pvParameters);
void wireCommunication(void *pvParameters);


bool goToState(int lines, bool isForwardDir, bool isRightStop, bool doTwoStepSlow);
void followLine(int maxSpeed, bool isForwardDir);
bool countLine(int lines, bool isForwardDir, bool isRight);
void readSideSensors(bool isForwardDir);
void motorPower(bool isForwardDir, uint32_t leftValue, uint32_t rightValue);
void centreRobot(bool isForwardDir);

void setCommPinOutput(int taskNumber);
int readCommPinInput();

TaskHandle_t executeTaskHandle = NULL; // Handle for the executeTask

void setup(){
  Serial.begin(115200);

  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);

  ledcSetup(L1, pwmFrequency, pwmResolution);
  ledcSetup(L2, pwmFrequency, pwmResolution);
  ledcSetup(R1, pwmFrequency, pwmResolution);
  ledcSetup(R2, pwmFrequency, pwmResolution);

  ledcAttachPin(motorL1, L1);
  ledcAttachPin(motorL2, L2);
  ledcAttachPin(motorR1, R1);
  ledcAttachPin(motorR2, R2);

  pinMode(signal, OUTPUT);  // DemonCore-Slave will control this pin
  pinMode(readyPin, INPUT);    // DemonCore-Slave will read this pin

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    wireCommunication,       // Task function
    "wireCommunication",     // Name of the task (for debugging)
    4096,             // Stack size (bytes)
    NULL,             // Parameter to pass to the task
    1,                // Task priority
    NULL,             // Task handle
    0                 // Core to run the task on (0 in this case)
  ); 
  
}

void loop(){
  vTaskDelete(NULL);
}

/**
 * @brief Handles wire communication and task execution based on received state.
 *
 * This function continuously monitors the `readyPin` to check if ESP-1 is ready to send data. If ESP-1 is ready, it reads the 
 * incoming state using `readCommPinInput()` and compares it with the previous state. If the state is valid and different from 
 * the previous state, it signals ESP-2 that the data has been received by setting the `signal` pin high. It then creates and 
 * runs the `executeTask` function on core 1. The function waits until `executeTask` is no longer running, indicating task completion.
 * Finally, it signals task completion to ESP-1 by setting the `signal` pin low and updates the `previousState` variable.
 *
 * @param pvParameters Pointer to task parameters (not used in this function).
 */
void wireCommunication(void *pvParameters) {
  while (true) {
    if (digitalRead(readyPin) == HIGH) { // Check if ESP-1 is ready
      state = readCommPinInput();
      
      if(state == previousState || state == 8){
        vTaskDelay(50);
      } 

      else {
        Serial.print("Received Task Number: ");
        Serial.println(state);
        digitalWrite(signal, HIGH); // Signal that ESP-2 has received the data

        // Create the executeTask
        xTaskCreatePinnedToCore(
            executeTask,         // Task function
            "executeTask",       // Name of the task (for debugging)
            4096,                // Stack size (bytes)
            NULL,                // Parameter to pass to the task
            1,                   // Task priority
            &executeTaskHandle,  // Task handle
            1                    // Core to run the task on (1 in this case)
        );

        // Wait until the executeTask is no longer running
        while (eTaskGetState(executeTaskHandle) != eDeleted) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        Serial.println("Task Complete");
        previousState = state;

        digitalWrite(signal, LOW); // Signal that the task is completed
        Serial.println("Task completed and signal sent back to ESP-1");
      }
    }
    // Wait a bit before the next read
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Executes tasks based on the current state.
 *
 * This function handles the execution of tasks corresponding to the current state. It prints the state and executes a specific 
 * task depending on the state. The tasks include moving to different stations and adjusting speed settings. After completing the task, 
 * the function updates the `ready` and `move` flags, and resets the speed settings if they were changed. Finally, the function 
 * deletes itself upon completion.
 *
 * @param pvParameters Pointer to task parameters (not used in this function).
 */
void executeTask(void *pvParameters){  
  Serial.println("Executing tasks...");
  delay(1000);
  switch(state){
    case 1: {
      stopSpeed = 4095;
      stopCount = 8;
      stopDelay = 10;
      slowSpeed = 1600;
      bool complete = goToState(firstStation, true, false, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }
    
    case 2: {
      slowSpeed = 1350;
      stopCount = 9;
      stopDelay = 9;
      bool complete = goToState(2,true, true, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 3: {
      slowSpeed += 50;
      stopCount = 12;
      stopDelay = 7;
      bool complete = goToState(1, false, false, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
        slowSpeed -= 50;
      }
      break;
    }

    case 4: {
      stopCount = 12;
      stopDelay = 8;
      bool complete = goToState(1, true, true, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 5: {
      slowSpeed += 50;
      stopCount = 12;
      stopDelay = 7;
      bool complete = goToState(1, false, false, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
        slowSpeed -= 50;
      }
      break;
    }

    case 6: {
      slowSpeed = 1250;
      regSpeed -= 200;
      stopSpeed = 3700;
      stopCount = 10;
      stopDelay = 10;
      bool complete = goToState(2, true, true, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 7: {
      stopCount = 8;
      stopDelay = 10;
      bool complete = goToState(3, false, false, true);
      regSpeed -= 100;
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
        firstStation = 0;
        slowSpeed = 1350;
        regSpeed += 300;
      }
      break;
    }
    default:
      break; 
  }
  Serial.println("Deleting Task executeTask");
  vTaskDelete(NULL);
}

/**
 * @brief Controls the robot's movement to reach a target state.
 *
 * This function directs the robot to move along a line based on the provided parameters. It adjusts the robot's speed and
 * direction depending on the proximity to the target line and sensor inputs. The function handles both forward and backward
 * movement scenarios and manages the stopping process by gradually reducing speed. After stopping, it centers the robot 
 * to ensure proper alignment.
 *
 * @param lines The target number of lines to go
 * @param isForwardDir Direction of movement (`true` for forward, `false` for backward).
 * @param isRightStop Flag indicating the side the robot stops on (`true` for right, `false` for left).
 * @param doTwoStepSlow Flag indicating if a two-step slowdown should be performed.
 *
 * @return `true` if the movement and stopping process was completed successfully.
 */
bool goToState(int lines, bool isForwardDir, bool isRightStop, bool doTwoStepSlow) {
  Serial.println("Moving...");
  currentLineCount = 0;
  almostThere = false;

  while (move) {
    Serial.println("In Moving...");

    bool lineClose = countLine(lines - 1, isForwardDir, isRightStop);
    if (lineClose) {
      almostThere = true;
    }

    if (!almostThere) {
      float speedReduction = (float)currentLineCount;
      if (speedReduction == 0 || speedReduction == 1) {
        speedReduction = 1;
      } else {
        speedReduction -= 0.4;
      }
      int speed = (int)(regSpeed / speedReduction);
      followLine(speed, isForwardDir);
    } else {
      readSideSensors(isForwardDir);
      if (isForwardDir) {
        if ((frontLeftDetected && !isRightStop) || (frontRightDetected && isRightStop)) {
          forwardDetected = true;
          if (doTwoStepSlow) {
            for (int i = 0; i <= 2; i++) {
              followLine(slowSpeed - 100, !isForwardDir);
              delay(10);
            }
          }
        }

        if (forwardDetected) {
          if (backLeftDetected || backRightDetected) {
            move = false;
            Serial.println("Stopping");
            forwardDetected = false;
            for (int i = 0; i <= stopCount; i++) {
              followLine(stopSpeed, !isForwardDir);
              delay(stopDelay);
            }
          }
        }
      } else {
        if ((backLeftDetected && !isRightStop) || (backRightDetected && isRightStop)) {
          forwardDetected = true;
          if (doTwoStepSlow) {
            for (int i = 0; i <= 2; i++) {
              followLine(slowSpeed - 100, !isForwardDir);
              delay(10);
            }
          }
        }

        if (forwardDetected) {
          if (frontLeftDetected || frontRightDetected) {
            move = false;
            Serial.println("Stopping");
            forwardDetected = false;
            for (int i = 0; i <= stopCount; i++) {
              followLine(stopSpeed, !isForwardDir);
              delay(stopDelay);
            }
          }
        }
      }

      if (isForwardDir) {
        followLine(slowSpeed, isForwardDir);
      } else {
        followLine(slowSpeed - 200, isForwardDir);
      }
    }
  }
  Serial.println("Centering Robot...");
  centreRobot(isForwardDir);
  return true;
}

/**
 * @brief Controls the robot's movement along a line using PID control.
 *
 * This function adjusts the speed of the robot's motors to follow a line based on sensor readings. It uses a PID controller 
 * to calculate the error between the left and right sensor readings, adjusting the motor speeds accordingly to correct the 
 * robot's path. The function supports both forward and backward movement directions with different PID coefficients. It also 
 * clamps the motor speeds to ensure they stay within valid PWM ranges and includes an optional debugging output for sensor 
 * readings and control values.
 *
 * @param maxSpeed The maximum speed for the motors.
 * @param isForwardDir Direction of movement (`true` for forward, `false` for backward).
 */
void followLine(int maxSpeed, bool isForwardDir) {
  int analogLeftValue, analogRightValue, leftValue, rightValue;
  if (isForwardDir) {
    analogLeftValue = analogRead(lineFrontLeft);
    analogRightValue = analogRead(lineFrontRight);
    Kp = 0.6;
    Ki = 0.25;
    Kd = 0.15;
  } 
  
  else {
    analogLeftValue = analogRead(lineBackLeft);
    analogRightValue = analogRead(lineBackRight);
    Kp = 0.75;
    Ki = 0.05;
    Kd = 0.4;
  }

  int error = analogLeftValue - analogRightValue;
  integral += error;
  float derivative = error - previousError;
  previousError = error;
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Limit the integral term to prevent windup
  float maxIntegral = 1000;
  if (integral > maxIntegral) {
      integral = maxIntegral;
  }
  else if (integral < -maxIntegral) {
    integral = -maxIntegral;
  }

  leftValue = maxSpeed - output;
  rightValue = maxSpeed + output;

  // Clamp values to the valid PWM range
  leftValue = constrain(leftValue, 0, maxSpeed);
  rightValue = constrain(rightValue, 0, maxSpeed);

  // Debugging output values
  {
  /*
  Serial.print("Analog Left: ");
  Serial.print(analogLeftValue);
  Serial.print(" | Analog Right: ");
  Serial.print(analogRightValue);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Integral: ");
  Serial.print(integral);
  Serial.print(" | Derivative: ");
  Serial.print(derivative);
  Serial.print(" | Output: ");
  Serial.print(output);
  Serial.print(" | Left Value: ");
  Serial.print(leftValue);
  Serial.print(" | Right Value: ");
  Serial.println(rightValue);
  */
  }

  motorPower(isForwardDir, leftValue, rightValue);
}

/* @brief Controls 4 PWM channels to drive 2 Motors
 *
 * @param isForwardDir Boolean to determine direction, forward towards castor wheel
 * @param leftValue An integer value to control speed of left motor
 * @param rightValue An integer value to control speed of right motor
*/
void motorPower(bool isForwardDir, uint32_t leftValue, uint32_t rightValue){
  if(isForwardDir){
      ledcWrite(L1, leftValue);
      ledcWrite(R1, rightValue);
      ledcWrite(L2, 0);
      ledcWrite(R2,0);
    }

    else{
      ledcWrite(L1,0);
      ledcWrite(R1,0);
      ledcWrite(L2,leftValue);
      ledcWrite(R2, rightValue);
    }
}

/**
 * @brief Counts the number of lines or waypoints detected by the robot.
 *
 * This function checks if the robot has detected a specified number of lines or waypoints based on sensor readings. It 
 * differentiates between forward and backward directions and updates the line count when the robot detects the end of a 
 * line segment. The function utilizes side sensors to detect line segments and updates the `currentLineCount` accordingly. 
 * It handles detection for both forward and backward movements and maintains the state of line detection.
 *
 * @param lines The target number of lines to be detected.
 * @param isForwardDir Direction of movement (`true` for forward, `false` for backward).
 * @param isRight Flag indicating whether to check the right side (`true` for right, `false` for left).
 *
 * @return `true` if the target number of lines has been detected; otherwise, returns `false`.
 */
bool countLine(int lines, bool isForwardDir, bool isRight) {
  //Serial.println(currentLineCount);

  if (currentLineCount == lines) {
    return true;
  }

  readSideSensors(isForwardDir);

  if (isForwardDir) {
    if ((isRight && frontRightDetected) || (!isRight && frontLeftDetected)) {
      if (!forwardDetected) {
        isLeft = isRight ? 2 : 1;
        forwardDetected = true;
      }
    }

    if (forwardDetected) {
      if ((isRight && backRightDetected) || (!isRight && backLeftDetected)) {
        switch (isLeft) {
          case 1:
            if (backLeftDetected) {
              currentLineCount++;
              forwardDetected = false;
              isLeft = 0;
            } else {
              forwardDetected = false;
              isLeft = 0;
            }
            break;

          case 2:
            if (backRightDetected) {
              currentLineCount++;
              forwardDetected = false;
              isLeft = 0;
            } else {
              forwardDetected = false;
              isLeft = 0;
            }
            break;

          default:
            break;
        }
      }
    }

  } else { // Backward direction
    if ((isRight && backRightDetected) || (!isRight && backLeftDetected)) {
      if (!forwardDetected) {
        isLeft = isRight ? 4 : 3;
        forwardDetected = true; // Update forwardDetected here
      }
    }

    if (forwardDetected) {
      if ((isRight && frontRightDetected) || (!isRight && frontLeftDetected)) {
        switch (isLeft) {
          case 3:
            if (frontLeftDetected) {
              currentLineCount++;
              forwardDetected = false;
              isLeft = 0;
            } else {
              forwardDetected = false;
              isLeft = 0;
            }
            break;

          case 4:
            if (frontRightDetected) {
              currentLineCount++;
              forwardDetected = false;
              isLeft = 0;
            } else {
              forwardDetected = false;
              isLeft = 0;
            }
            break;

          default:
            break;
        }
      }
    }
  }

  return false;
}

/**
 * @brief Centers the robot by aligning it based on sensor inputs.
 *
 * This function stops the robot and uses side sensors to adjust its position to the center. It checks the sensor readings
 * for both forward and backward directions and adjusts the robot's movement accordingly to ensure it is properly centered.
 * The function includes a timeout mechanism to prevent indefinite operation and small delays to avoid overshooting.
 *
 * @param isForwardDir Direction of movement (`true` for forward, `false` for backward).
 */
void centreRobot(bool isForwardDir) {
  Serial.println("Centering...");
  // Stop the robot before centering
  motorPower(isForwardDir, 0, 0);
  readSideSensors(isForwardDir);

  long time = millis();
  while (true) {
    if (millis() - time > centreTime) {
      break;
    }

    readSideSensors(isForwardDir);

    if (isForwardDir) {
      // Move backward until the front sensors are detected
      if (!frontLeftDetected && !frontRightDetected) {
        followLine(1300, false);
      } else if (frontLeftDetected || frontRightDetected) {
        followLine(1300, true);
      }
    } else {
      // Move forward until the back sensors are detected
      if (!backLeftDetected && !backRightDetected) {
        followLine(1300, true);
      } else if (backLeftDetected || backRightDetected) {
        followLine(1300, false);
      }
    }

    // Stop centering when both front and back detections are false
    if ((!backLeftDetected && !backRightDetected && !frontLeftDetected && !frontRightDetected)) {
      break;
    }

    delay(10);  // Add a small delay to avoid overshooting
  }

  motorPower(isForwardDir, 0, 0);
}

/**
 * @brief Reads values from side sensors and updates their detected status.
 *
 * This function reads the analog values from four side sensors (front left, back left, front right, and back right). 
 * It then compares these values against a predefined threshold and updates the corresponding boolean flags to indicate 
 * whether an object has been detected by each sensor. This information is used to help navigate the robot.
 *
 * @param isForwardDir Indicates the direction of movement. If true, the robot is moving forward; if false, it is moving backward.
 */
void readSideSensors(bool isForwardDir){
  //Read value from sensors
  int frontLeftValue = analogRead(frontLeftSensor);
  int backLeftValue = analogRead(backLeftSensor);
  int frontRightValue = analogRead(frontRightSensor);
  int backRightValue = analogRead(backRightSensor);

//Debugging Output
  /*
  Serial.print("Front Left Value: ");
  Serial.println(frontLeftValue);
  Serial.print("Front Right Value: ");
  Serial.println(frontRightValue);
  Serial.print("Back Left Value: ");
  Serial.println(backLeftValue);
  Serial.print("Back Right Value: ");
  Serial.println(backRightValue);
  */

  if(frontLeftValue > OPTICAL_SENSOR_THRESHOLD){
    frontLeftDetected = true;
    //Serial.println("Front Left Detected: True");
  }
  else{
    frontLeftDetected = false;
    //Serial.println("Front Left Detected: False");
  }
  
  if(frontRightValue > OPTICAL_SENSOR_THRESHOLD){
    frontRightDetected = true;
    //Serial.println("Front Right Detected: True");
  }
  else {
    frontRightDetected = false;
    //Serial.println("Front Right Detected: False");
  }

  if(backLeftValue > OPTICAL_SENSOR_THRESHOLD){
    backLeftDetected = true;
    //Serial.println("Back Left Detected: True");
  }
  else{
    backLeftDetected = false;
    //Serial.println("Back Left Detected: False");
  }

  if(backRightValue > OPTICAL_SENSOR_THRESHOLD){
    backRightDetected = true;
    //Serial.println("Back Right Detected: True");
  }
  else{
    backRightDetected = false;
    //Serial.println("Back Right Detected: False");
  }
}

/* @brief Converts integer into binary
 *
 * This method converts an integer value into a binary value to be send down communication pins
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
 * The method reads the communication pins and converts the binary signal into integer value
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
