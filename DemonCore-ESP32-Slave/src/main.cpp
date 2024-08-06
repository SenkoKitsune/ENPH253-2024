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

//Global variables for counting rotations
int currentRotationCount = 0;

int firstStation = 1;

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

const int slowSpeed = 1250;
int stoppingTime = 80;

// PID control constants
float Kp = 0;
float Ki = 0;
float Kd = 0;

float previousError = 0;
float integral = 0;

//forward declaration of methods
void executeTask(void *pvParameters);
void countRotation(void *pvParameters);
void wireCommunication(void *pvParameters);


bool goToState(int rotations, int lines, bool isForwardDir, bool isRightStop);
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


void executeTask(void *pvParameters){  
  Serial.println("Executing tasks...");
  delay(1000);
  switch(state){
    case 1: {
      bool complete = goToState(12, firstStation, true, false);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }
    
    case 2: {
      bool complete = goToState(12,2,true, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 3: {
      bool complete = goToState(12, 1, false, false);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 4: {
      bool complete = goToState(12, 1, true, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 5: {
      bool complete = goToState(12, 1, false, false);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
      }
      break;
    }

    case 6: {
      stoppingTime = 130;
      bool complete = goToState(12, 2, true, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
        stoppingTime = 100;
      }
      break;
    }

    case 7: {
      bool complete = goToState(12, 3, false, false);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        move = true;
        firstStation = 0;
      }
      break;
    }
    default:
      break; 
  }
  Serial.println("Deleting Task executeTask");
  vTaskDelete(NULL);
}

bool goToState(int rotations, int lines, bool isForwardDir, bool isRightStop){
  Serial.println("Moving...");
  currentLineCount = 0;
  currentRotationCount = 0;
  almostThere = false;

  while(move){
  Serial.println("In Moving...");
    /*
    xTaskCreatePinnedToCore(
      countRotation,       // Task function
      "countRotation",     // Name of the task (for debugging)
      4096,             // Stack size (bytes)
      (void *)&lines,   // Parameter to pass to the task
      1,                // Task priority
      NULL,             // Task handle
      0                 // Core to run the task on (0 in this case)
    );  
    */

    
    bool lineClose = countLine(lines - 1, isForwardDir, isRightStop);
    if(lineClose){
      almostThere = true;
    }

    if(!almostThere && currentRotationCount < rotations){
      float speedReduction = (float) currentLineCount;
      if(speedReduction == 0){
        speedReduction = 1;
      }
      else{
        speedReduction += 0.5;
      }
      int speed = (int) (2459 / speedReduction);
      followLine(speed, isForwardDir);
    }

    else{
      readSideSensors(isForwardDir);
      if(isForwardDir){
        if(frontLeftDetected && !isRightStop){
          forwardDetected = true;
          for(int i = 0; i <= 2; i++){
              followLine(slowSpeed, !isForwardDir);
              delay(11);
            }
        }
        else if(frontRightDetected && isRightStop){
          forwardDetected = true;
          for(int i = 0; i <= 2; i++){
              followLine(slowSpeed, !isForwardDir);
              delay(11);
            }
        }

        if(forwardDetected){
          if(backLeftDetected || backRightDetected){
            move = false;
            Serial.println("Stopping");
            forwardDetected = false;
            int stopSpeed = 4095;
            for(int i = 0; i <= 7; i++){
              followLine(stopSpeed, !isForwardDir);
              stopSpeed -= 120;
              delay(11);
            }
          }
        }
      }

      else{
        if(backLeftDetected && !isRightStop){
          forwardDetected = true;
          for(int i = 0; i <= 2; i++){
              followLine(slowSpeed, !isForwardDir);
              delay(11);
            }
        }
        else if(backRightDetected && isRightStop){
          forwardDetected = true;
          for(int i = 0; i <= 2; i++){
              followLine(slowSpeed, !isForwardDir);
              delay(11);
            }
        }

        if(forwardDetected){
          if(frontLeftDetected || frontRightDetected){
            move = false;
            Serial.println("Stopping");
            forwardDetected = false;
            int stopSpeed = 4095;
            for(int i = 0; i <= 8; i++){
              followLine(stopSpeed, !isForwardDir);
              stopSpeed -= 100;
              delay(11);
            }
          }
        }
      }

      followLine(slowSpeed, isForwardDir);
    }
  }
  Serial.println("Centering Robot...");
  centreRobot(isForwardDir);
  return true;
}


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
    Ki = 0;
    Kd = 0.3;
  }


int error = analogLeftValue - analogRightValue;
integral += error;
float derivative = error - previousError;
previousError = error;

float output = Kp * error + Ki * integral + Kd * derivative;

// Optional: Limit the integral term to prevent windup
float maxIntegral = 1000; // Adjust as necessary
if (integral > maxIntegral) {
    integral = maxIntegral;
} else if (integral < -maxIntegral) {
    integral = -maxIntegral;
}

// Use output as needed


  leftValue = maxSpeed - output;
  rightValue = maxSpeed + output;

  // Clamp values to the valid PWM range
  leftValue = constrain(leftValue, 0, maxSpeed);
  rightValue = constrain(rightValue, 0, maxSpeed);

  // Debugging output values
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


bool countLine(int lines, bool isForwardDir, bool isRight) {
  Serial.println(currentLineCount);

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


void centreRobot(bool isForwardDir) {
  // Stop the robot before centring
  motorPower(isForwardDir, 0, 0);
  delay(500); // Allow time for the robot to come to a complete stop

  readSideSensors(isForwardDir);
  
  if (isForwardDir) {
    while (backLeftDetected || backRightDetected || frontLeftDetected || frontRightDetected) {
      readSideSensors(isForwardDir);
      if (backLeftDetected || backRightDetected) {
        followLine(1400, !isForwardDir);
      } else if (frontLeftDetected || frontRightDetected) {
        followLine(1400, isForwardDir);
      }
      delay(10);  // Add a small delay to avoid overshooting
    }
  } else {
    while (backLeftDetected || backRightDetected || frontLeftDetected || frontRightDetected) {
      readSideSensors(isForwardDir);
      if (backLeftDetected) {
        followLine(1500, isForwardDir);
      } else if (frontLeftDetected) {
        followLine(1500, !isForwardDir);
      }
      delay(10);  // Add a small delay to avoid overshooting
    }
  }

  motorPower(isForwardDir, 0, 0);
}


void readSideSensors(bool isForwardDir){
  int frontLeftValue = analogRead(frontLeftSensor);
  int backLeftValue = analogRead(backLeftSensor);
  int frontRightValue = analogRead(frontRightSensor);
  int backRightValue = analogRead(backRightSensor);

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


void countRotation(void *pvParameters){
  //Cast void into int
  int rotations = *(int *)pvParameters;
  Serial.println(rotations);
  vTaskDelete(NULL);
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
