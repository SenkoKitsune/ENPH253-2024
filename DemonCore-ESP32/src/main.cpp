#include "Arduino.h"
#include "WiFi.h"
#include "AsyncTCP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <ESP32Servo.h>

const char* ssid = "U235-Control";
const char* password = "SkibidiToilet";
const char* server_ip = "192.168.15.221";
const uint16_t server_port = 80;

bool doBurger = true;
bool proceed = false;

//Wire communication pins
#define commBit0 21
#define commBit1 22
#define commBit2 19
#define commBit3 8
#define signal 7
#define ready 5

//Burger arm pins
#define armServoPin 27
#define turnServoPin 33
#define spatulaMotor1 2 
#define spatulaMotor2 4 

//Servo number assignment
#define armServoNo 1
#define turnServoNo 2

// Define initial speed and delay parameters for smooth servo control
const int minDelay = 30;      // Minimum delay in milliseconds
const int maxDelay = 50;     // Maximum delay in milliseconds
int currentArmServoPos = 40; // Start from the middle position
int currentTurnServoPos = 100;

// Timing for spatula open/close
const int spatulaTime = 1100;

//Object creation for WiFi
AsyncClient client;
QueueHandle_t commandQueue;

//FreeRTOS task declaration:

void TCP_Client(void *pvParameters);
void ExecuteTasks(void *pvParameters);

//Burger task delcaration

void burgerTask1();
void burgerTask2();
void burgerTask3();
void burgerTask4();
void burgerTask5();
void burgerTask6();
void burgerTask7();
void burgerTask8();
void burgerTask9();
void burgerTask10();
void burgerTask11();
void burgerTask12();
void burgerTask13();

//Wire communication

void setCommPinOutput(int taskNumber);
int readCommPinInput();
void waitForSignal(int pin);
void performTask(int taskNo);

//Spatula control

void moveSpatula(bool close);
void stopSpatula();
void controlSpatula(int taskNo);

//Smooth servo control
Servo armServo;
Servo turnServo;
float easeInOutCubicSlow(float t);
void smoothServoControl(int endPos, int servoNo);

//Common angles declaration
const int leftCounter = 180;
const int centre = 100;
const int rightCounter = 10;
const int onCounter = 30;
const int flatSurface = 43;
const int armUp = 180;

void setup() {
  Serial.begin(115200);

  //Initialise Spatula motors
  pinMode(spatulaMotor1, OUTPUT);
  pinMode(spatulaMotor2, OUTPUT);

  // Connect to ESP32 AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to ESP32 AP...");
  }
  // Create FreeRTOS queue
  commandQueue = xQueueCreate(20, sizeof(char[70]));
  Serial.println("Connected to ESP32 AP");

  //Initialise Servo motors
  armServo.attach(armServoPin, 500, 2500); // Min pulse width, Max pulse width
  turnServo.attach(turnServoPin, 500, 2500);
  armServo.write(currentArmServoPos);
  turnServo.write(currentTurnServoPos);
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    TCP_Client,       // Task function
    "TCP Client",     // Name of the task (for debugging)
    8192,             // Stack size (bytes)
    NULL,             // Parameter to pass to the task
    1,                // Task priority
    NULL,             // Task handle
    0                 // Core to run the task on (0 in this case)
  );

  xTaskCreatePinnedToCore(
    ExecuteTasks,
    "Execute Tasks",
    8192,
    NULL,
    1,                // Task priority
    NULL,
    1                 // Core to run the task on (1 in this case)
  );  
}

void loop() {
  vTaskDelete(NULL);
}

/**
 * @brief Manages a TCP client connection on the ESP32
 * 
 * @param pvParameters A void pointer to the parameters passed to the task. Unused in this method
 * 
 **/
void TCP_Client(void *pvParameters) {
  client.onConnect([](void *s, AsyncClient* c) {
    Serial.println("Connected to server");
  });

  client.onDisconnect([](void *s, AsyncClient* c) {
    Serial.println("Disconnected from server");
  });

  client.onData([](void *s, AsyncClient* c, void *data, size_t len) {
    char command[50];
    strncpy(command, (char*)data, len);
    command[len] = '\0';
    Serial.printf("Received command from server: %s\n", command);
    if(strcmp(command, "Action Received")){
      proceed = true;
    }
    xQueueSend(commandQueue, &command, portMAX_DELAY);
  });

  if (!client.connected()) {
    client.connect(server_ip, server_port);
  }

  while (true) {
    // Task keeps checking for connectivity and responses
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as needed
  }
}

/**
 * @brief This method executes tasks on core 1 of the ESP-32. 
 * 
 * @param pvParameters A void pointer to the parameters passed to the task. Unused in this method
 */
void ExecuteTasks(void *pvParameters) {
  /*
   * The doBurger loop. After finishing the fries step, the robot will repeat the burger loop until we run out of time
   * It will do its own loop until it needs to do the burger plating step. The burger will then send communication wirelessly to 
   * the Screwdriver control ESP-32.
  */
  while (doBurger) {
    // Execute tasks 0 to 13
    for (int i = 1; i < 13; i++) {
      switch (i) {
        case 1: burgerTask1(); break;
        case 2: burgerTask2(); break;
        case 3: burgerTask3(); break;
        case 4: burgerTask4(); break;
        case 5: burgerTask5(); break;
        case 6: burgerTask6(); break;
        case 7: burgerTask7(); break;
        case 8: burgerTask8(); break;
        case 9: burgerTask9(); break;
        case 10: burgerTask10(); break;
        case 11: burgerTask11(); break; 
        case 12: {
          burgerTask12();
          // After task 12, send message to server and wait for response
          // Message to server acts like a "I am here"
          // Response from server acts like "I have arrived"
          const char* instruction = "Reached Task 12";
          client.write(instruction, strlen(instruction));
          Serial.printf("Sent instruction: %s\n", instruction);

          // Wait for the command to execute task 13
          char command[50];
          while (true) {
            if (xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
              if (strcmp(command, "Execute Task 13") == 0) {
                break;
              }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as needed
          }

          // Execute task 13
          burgerTask13();
          instruction = "Finished Task 13";
          client.write(instruction, strlen(instruction));
          Serial.printf("Sent instruction: %s\n", instruction);

          //Wait for the server to confirm action
          while (!proceed) {
            delay(100);
          }
          delay(100);
          vTaskDelete(NULL);
          break;
        }  
      }
    }
  }
}

/**
 * @brief Goes towards patty station, move arms to left counter
 */
void burgerTask1() { 
  Serial.println("Executing burger task 1"); 
  performTask(1);
}

/**
 * @brief Controls servo arm to pick up patty
 */
void burgerTask2() { 
  Serial.println("Executing burger task 2"); 
  smoothServoControl(onCounter, 1);
  controlSpatula(1);
  smoothServoControl(flatSurface, 1);
  //smoothServoControl(centre, 2);
}

/**
 * @brief Moves robot to cooking station
 */
void burgerTask3() { 
  Serial.println("Executing burger task 3");
  performTask(2);
}

/**
 * @brief Controls servo arm to drop the patty on the cooking station
 */
void burgerTask4() { 
  Serial.println("Executing burger task 4"); 
  smoothServoControl(onCounter, 1);
  controlSpatula(2);
  smoothServoControl(flatSurface, 1);
  //smoothServoControl(centre, 2);
}

/**
 * @brief Moves robot to bun station for top bun
 */
void burgerTask5() { 
  Serial.println("Executing burger task 5"); 
  performTask(3);
}

/**
 * @brief Controls servo arm to pick up top bun
 */
void burgerTask6() { 
  Serial.println("Executing burger task 6"); 
  controlSpatula(2);
  smoothServoControl(onCounter, 1);
  controlSpatula(1);
  smoothServoControl(flatSurface, 1);
  //smoothServoControl(centre, 2);
}

/**
 * @brief Moves robot to cooking station
 */
void burgerTask7() { 
  Serial.println("Executing burger task 7"); 
  performTask(4);
}

/**
 * @brief Picks up patty from cooking station
 */
void burgerTask8() { 
  Serial.println("Executing burger task 8"); 
  controlSpatula(2);
  smoothServoControl(onCounter,1);
  controlSpatula(1);
  smoothServoControl(flatSurface, 1);
  //smoothServoControl(centre, 2);
}

/**
 * @brief Moves robot to bun station from cooking station to pick up top bun
 */
void burgerTask9() { 
  Serial.println("Executing burger task 9"); 
  performTask(5);
}

/**
 * @brief Controls servo arm to pick up bottom bun
 */
void burgerTask10() { 
  Serial.println("Executing burger task 10"); 
  controlSpatula(2);
  smoothServoControl(onCounter, 1);
  controlSpatula(1);
  smoothServoControl(flatSurface, 1);
}

/**
 * @brief Moves robot to serving area, moves arm high above plate
 */
void burgerTask11() { 
  Serial.println("Executing burger task 11"); 
  performTask(6); 
}

/**
 * @brief Controls servo arm to drop burger on plate
 */
void burgerTask12() { 
  Serial.println("Executing burger task 12"); 
  smoothServoControl(onCounter, 1);
  controlSpatula(2);
  smoothServoControl(130,1);
}

/**
 * @brief Reset robot position to patty station
 */
void burgerTask13() { 
  Serial.println("Executing burger task 13"); 
  performTask(7);
}

/**
 * @brief Cubic easing function with slower easing-in.
 *
 * This function provides a cubic easing calculation, modified to have a slower easing-in effect. 
 * It smoothly interpolates the progress `t` over a duration, using a cubic equation for a more 
 * gradual start and end. The function is typically used for animations or movements that require 
 * a smooth, non-linear progression.
 *
 * @param t A float representing the normalized time (progress) in the range [0, 1].
 * @return A float representing the eased progress.
 */
float easeInOutCubicSlow(float t) {
  t *= 2;
  if (t < 1) {
    return 0.5 * (t * t * t);
  }
  t -= 2;
  return 0.5 * (t * t * t + 2);
}

/**
 * @brief Controls a servo motor with cubic easing.
 *
 * This function moves a specified servo motor to a target position using a cubic easing function 
 * for smooth acceleration and deceleration. It calculates the required position and delay for 
 * each step, ensuring a smooth and gradual movement from the current position to the target 
 * position. The easing function is used to interpolate the servo position, providing a smoother 
 * transition.
 *
 * @param endPos The absolute target position (0 <= pos <= 180) to move the servo to.
 * @param servoNo The identifier for the servo to move (1 for armServo, 2 for turnServo).
 */
void smoothServoControl(int endPos, int servoNo) {
  int startPos;
  switch (servoNo) {
  case 1:
    startPos = currentArmServoPos;
    break;
  case 2:
    startPos = currentTurnServoPos;
    break;
  default:
    return; // Exit if servoNo is invalid
  }
  
  int range = abs(endPos - startPos);
  int increment = (startPos < endPos) ? 1 : -1;

  for (int i = 0; i <= range; i++) {
    float progress = (float)i / range;
    float easedProgress = easeInOutCubicSlow(progress);
    int pos = startPos + increment * (easedProgress * range);
    switch (servoNo) {
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

  switch (servoNo) {
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

/**
 * @brief Perform a predetermined task while communicating over a wire connection.
 *
 * This function executes a specific task based on the provided task number, adjusting servo positions 
 * accordingly and communicating with another microcontroller over a wire connection. It handles the 
 * necessary communication signaling to ensure synchronized task execution.
 *
 * @param taskNo The task number to execute. This determines the specific actions to perform, 
 * such as setting the angles for the arm and turn servos.
 */
void performTask(int taskNo) {
  Serial.println("Performing Tasks");
  int armAngle;
  int turnAngle;
  switch (taskNo) {
  case 1: {
    armAngle = flatSurface;
    turnAngle = leftCounter;
    break;
  }
  case 2: {
    armAngle = flatSurface;
    turnAngle = rightCounter;
    break;
  }
  case 3: {
    armAngle = flatSurface;
    turnAngle = leftCounter;
    break;
  }
  case 4: {
    armAngle = flatSurface;
    turnAngle = rightCounter;
    break;
  }
  case 5: {
    armAngle = flatSurface + 5;
    turnAngle = leftCounter;
    break;
  }
  case 6: {
    armAngle = flatSurface + 30;
    turnAngle = rightCounter;
    break;
  }
  case 7: {
    armAngle = flatSurface;
    turnAngle = centre;
    break;
  }
  default:
    return; // Exit if taskNo is invalid
  }

  setCommPinOutput(taskNo);
  Serial.print("Task Number Sent: ");
  Serial.println(taskNo);

  digitalWrite(ready, HIGH);  // Signal ESP-2 that data is ready
  delay(100);
  if (armAngle != currentArmServoPos) {
    smoothServoControl(armAngle, 1);
  }

  if (turnAngle != currentTurnServoPos) {
    smoothServoControl(turnAngle, 2);
    delay(100); // some delay for turning
  }

  waitForSignal(signal);  // Wait for ESP-2 to complete the task
  digitalWrite(ready, LOW);  // Reset the ready signal

  setCommPinOutput(0); // Reset the output after receiving the signal from ESP-2
  Serial.println("Received completion signal from ESP-2");
}

/**
 * @brief Waits for a pin to go LOW from HIGH.
 *
 * This function continuously checks the state of a specified pin and waits until it goes LOW. 
 * It is used to synchronize actions with external signals, ensuring that the program proceeds 
 * only when the expected signal is received.
 *
 * @param pin The pin number to read. The function will wait until this pin's state changes from HIGH to LOW.
 */
void waitForSignal(int pin) {
  while (digitalRead(pin) == HIGH) {
    Serial.println("Waiting for signal to go LOW...");
    delay(10);
  }
  Serial.println("Signal is LOW, proceeding...");
}

/*
 *@brief Method to open and close the spatula
 * 
 * @param close true for closing action, false for opening action
 */
void moveSpatula(bool close) {
  if (close) {
    digitalWrite(spatulaMotor1, HIGH);
    digitalWrite(spatulaMotor2, LOW);
  } 
  else {
    digitalWrite(spatulaMotor1, LOW);
    digitalWrite(spatulaMotor2, HIGH);
  }
}

/*
 * @brief Method to stop operating the spatula motor
 */
void stopSpatula() {
  digitalWrite(spatulaMotor1, LOW);
  digitalWrite(spatulaMotor2, LOW);
}

/*
 *@brief Controls the opening and closing of the spatula, timed
 * 
 * @param taskNo 1 for opening, 2 for closing
 */
void controlSpatula(int taskNo){
  switch (taskNo)
  {
  case 1:
    moveSpatula(true);
    delay(spatulaTime);
    stopSpatula();
    break;

  case 2:
    moveSpatula(false);
    delay(spatulaTime);
    stopSpatula();
    break; 

  default:
    break;
  }
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
