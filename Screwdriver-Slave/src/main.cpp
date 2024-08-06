#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define lineFrontLeft 39
#define lineFrontRight 36
#define lineBackLeft 32
#define lineBackRight 33

#define frontLeftSensor 34
#define backLeftSensor 35

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
bool frontLeftDetected = false;
bool backLeftDetected = false;

bool forwardDetected = false;

//Global variables for counting rotations
int currentRotationCount = 0;

// PWM configuration
const int pwmFrequency = 100; // PWM frequency in Hz
const int pwmResolution = 12; // PWM resolution (1-16 bits)

//Motor pin declaration
const int motorL1 = 15;
const int motorL2 = 13;
const int motorR1 = 2;
const int motorR2 = 4;

//Assigning each motor lead individual PWM channels
const int L1 = 0;
const int L2 = 2;
const int R1 = 1;
const int R2 = 3;

//Constants for line detection
const int OPTICAL_SENSOR_THRESHOLD = 500;

const centreTime = 500;

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


bool goToState(int rotations, int lines, bool isForwardDir);
void followLine(int maxSpeed, bool isForwardDir);
bool countLine(int lines, bool isForwardDir);
void readSideSensors(bool isForwardDir);
void motorPower(bool isForwardDir, uint32_t leftValue, uint32_t rightValue);

void centreRobot(bool isForward);

void setCommPinOutput(int taskNumber);
int readCommPinInput();

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
  delay(50);
  while (true) {
    if (digitalRead(readyPin) == HIGH) { // Check if ESP-1 is ready
      state = readCommPinInput();
      Serial.print("Received Task Number: ");
      Serial.println(state);
      if(state == previousState || state == 8){
        Serial.println("Bad State");
        vTaskDelay(50);
      }
      
      else{
        digitalWrite(signal, HIGH); // Signal that ESP-2 has received the data

        xTaskCreatePinnedToCore(
            executeTask,     // Task function
            "executeTask",   // Name of the task (for debugging)
            4096,            // Stack size (bytes)
            NULL,            // Parameter to pass to the task
            1,               // Task priority
            NULL,            // Task handle
            1                // Core to run the task on (0 in this case)
        );

        while (!ready) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        ready = false;
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
  bool complete = false;
  switch(state){
    case 0: {
      complete = goToState(12,2,true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        delay(1000);
        move = true;
      }
      break;
    }
    case 1:{
      complete = goToState(12, 1, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        delay(1000);
        move = true;
      }
      break;
    }
    case 2: {
      complete = goToState(12, 1, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        delay(1000);
        move = true;
      }
      break;
    }
    case 3: {
      int rotations = 200;
      complete = goToState(rotations, 2, false);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        delay(1000);
        move = true;
      }
      break;
    }
    case 4: {
      complete = goToState(12, 2, true);
      if(complete){
        Serial.print("Complete round: ");
        Serial.println(state);
        ready = true;
        delay(1000);
        move = true;
      }
      break;
    }
    default:
      break; 
  }

  vTaskDelete(NULL);
}

bool goToState(int rotations, int lines, bool isForwardDir){ 
  currentLineCount = 0;
  currentRotationCount = 0;
  almostThere = false;

  while(move){
    //Serial.println(currentLineCount);
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
    
    bool lineClose = countLine(lines - 1, isForwardDir);
    if(lineClose){
      almostThere = true;
    }

    if(!almostThere && currentRotationCount < rotations){
      int speedReduction = currentLineCount;
      if(speedReduction == 0){
        speedReduction = 1;
      }
      int speed = (int) (2047 / speedReduction);
      followLine(speed, isForwardDir);
    }

    else{
      readSideSensors(isForwardDir);
      if(isForwardDir){
        if(frontLeftDetected){
          currentLineCount++;
          followLine(4095, !isForwardDir);
          delay(100);
          move = false;
          Serial.println("Stopping");
        }
      }

      else{
        if(backLeftDetected){
          currentLineCount++;
          followLine(4095, !isForwardDir);
          delay(100);
          move = false;
          Serial.println("Stopping");
        }
      }
      followLine(700, isForwardDir);
    }
  }
  centreRobot(isForwardDir);
  return true;
}


void followLine(int maxSpeed, bool isForwardDir) {
  int analogLeftValue, analogRightValue, leftValue, rightValue;
  if (isForwardDir) {
    analogLeftValue = analogRead(lineFrontLeft);
    analogRightValue = analogRead(lineFrontRight);
    Kp = 0.5;
    Ki = 0.25;
    Kd = 0.15;
  } 
  
  else {
    analogLeftValue = analogRead(lineBackLeft);
    analogRightValue = analogRead(lineBackRight);
    Kp = 0.5;
    Ki = 0.25;
    Kd = 0.15;
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


bool countLine(int lines, bool isForwardDir){
  /*
  Serial.print("Front Left Detected: ");
  Serial.println(frontLeftDetected);
  Serial.print("Forward Detected: ");
  Serial.println(forwardDetected);
  Serial.print("Back Left Detected: ");
  Serial.println(backLeftDetected);
  */
  //Serial.println(currentLineCount);
  if(currentLineCount == lines){
    return true;
  }

  readSideSensors(isForwardDir);

  if(isForwardDir){
    if(frontLeftDetected ){
      if(!forwardDetected){
        forwardDetected = true;
      }
    }

    if(forwardDetected){
      if(backLeftDetected){
          currentLineCount++;
          forwardDetected = false;
      }
    }
  }
  
  else{
    if(backLeftDetected){
      if(!forwardDetected){
        forwardDetected = true;
      }
    }

    if(forwardDetected){
      if(frontLeftDetected){
        currentLineCount++;
        forwardDetected = false;
      }
    }
  }
 
  return false;
}


void readSideSensors(bool isForwardDir){
  int frontLeftValue = analogRead(frontLeftSensor);
  int backLeftValue = analogRead(backLeftSensor);

  /*
  Serial.print("Front Left Value: ");
  Serial.println(frontLeftValue);
  Serial.print("Back Left Value: ");
  Serial.println(backLeftValue);
  delay(500);
  */

  if(frontLeftValue > OPTICAL_SENSOR_THRESHOLD){
    frontLeftDetected = true;
    //Serial.println("Front Left Detected: True");
  }
  else{
    frontLeftDetected = false;
    //Serial.println("Front Left Detected: False");
  }
  

  if(backLeftValue > OPTICAL_SENSOR_THRESHOLD){
    backLeftDetected = true;
    //Serial.println("Back Left Detected: True");
  }
  else{
    backLeftDetected = false;
    //Serial.println("Back Left Detected: False");
  }
}


void countRotation(void *pvParameters){
  //Cast void into int
  int rotations = *(int *)pvParameters;
  Serial.println(rotations);
  vTaskDelete(NULL);
}

void centreRobot(bool isForwardDir){
  long time = millis();
  readSideSensors(isForwardDir);
  if(isForwardDir){
    if(backLeftDetected){
      while(backLeftDetected){
        if(millis() - time < centreTime){
          break;
        }
        readSideSensors(isForwardDir);
        followLine(1023, !isForwardDir);
      }
    }
    else if(frontLeftDetected){
      while (frontLeftDetected){
        if(millis() - time < centreTime){
          break;
        }
        readSideSensors(isForwardDir);
        
        followLine(1023, isForwardDir);
      }
    }
  }
  else{
    if(backLeftDetected){
      while(backLeftDetected){
        if(millis() - time < centreTime){
          break;
        }
        readSideSensors(isForwardDir);
        followLine(1023, isForwardDir);
      }
    }
    else if(frontLeftDetected){
      while(frontLeftDetected){
        if(millis() - time < centreTime){
          break;
        }
        readSideSensors(isForwardDir);
        followLine(1023, !isForwardDir);
      }
    }
  }
  motorPower(isForwardDir, 0,0);
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
