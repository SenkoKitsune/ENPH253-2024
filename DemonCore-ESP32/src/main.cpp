#include "Arduino.h"
#include "WiFi.h"
#include "AsyncTCP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <ESP32Servo.h>

const char* ssid = "U235-Control";  // Replace with your AP's SSID
const char* password = "SkibidiToilet";  // Replace with your AP's password
const char* server_ip = "192.168.15.221";  // Default IP for ESP32 AP
const uint16_t server_port = 80;  // Replace with your server's port
bool doBurger = false;
bool proceed = false;

#define commBit0 21
#define commBit1 22
#define commBit2 19
#define commBit3 8
#define signal 7
#define ready 5

#define armServoPin 27
#define turnServoPin 33

// Define initial speed and delay parameters for smooth servo control
int minDelay = 30;      // Minimum delay in milliseconds
int maxDelay = 50;     // Maximum delay in milliseconds
int currentArmServoPos = 30; // Start from the middle position
int currentTurnServoPos = 100;

AsyncClient client;
QueueHandle_t commandQueue;
Servo armServo;
Servo turnServo;

// Forward declaration of tasks
void TCP_Client(void *pvParameters);
void ExecuteTasks(void *pvParameters);
void burgerTask0();
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
void burgerTask14();
void burgerTask15();
void burgerTask16();
void friesTask0();
void friesTask1();
void friesTask2();
void friesTask3();
void friesTask4();
void friesTask5();
void friesTask6(); 
void friesTask7();

void setCommPinOutput(int taskNumber);
int readCommPinInput();
void waitForSignal(int pin);
void performTask(int taskNo);

float easeInOutCubicSlow(float t);
void smoothServoControl(int endPos, int servoNo);

void setup() {
  Serial.begin(115200);

  // Connect to ESP32 AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to ESP32 AP...");
  }
  Serial.println("Connected to ESP32 AP");

  // Create FreeRTOS queue
  commandQueue = xQueueCreate(20, sizeof(char[70]));

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
  // Main loop can run other tasks if needed
}

// Task for handling TCP client communication on core 0
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

/*
 * This method executes tasks on core 1 of the ESP-32. It has two components, a one-time-use doFries loop and a multi-use doBurger loop.
 * It is also responsible for the initial sending of information when the robot has finished up to the hand-off step each loop.
 */
void ExecuteTasks(void *pvParameters) {

/*
 * The fries loop. It will do its own thing until the fries hand-off step, which will then require the two robots to communicate
 * with each other. Afterwards, the doBurgers boolean will be set to true and the doFries loop will be ignored.
 */
  if(!doBurger){
    for(int i = 0; i < 6; i++){
      switch(i) {
        case 0: friesTask0(); break;
        case 1: friesTask1(); break;
        case 2: friesTask2(); break;
        case 3: friesTask3(); break;
        case 4: friesTask4(); break;
        case 5: 
        {
          friesTask5();
          // After doing task 5, we wait for Screwdriver robot to show up so we can hand off the fries
          const char* instruction = "Reached Fries Task 5";
          client.write(instruction, strlen(instruction));
          Serial.printf("Sent instruction: %s\n", instruction);

          // Wait for the command to execute task 6
          char command[50];
          while (true) {
            if (xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
              if (strcmp(command, "Execute Task 6") == 0) {
                delay(50);
                break;
              }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as needed
          }

          // Execute task 6
          friesTask6();
          instruction = "Finished Task 6";
          client.write(instruction, strlen(instruction));
          Serial.printf("Sent instruction: %s\n", instruction);

          //Wait for the server to confirm action
          while (!proceed) {
            delay(100);
          }
          delay(100);
          friesTask7();
          break;
        }

      }
    }
    doBurger = true;
    proceed = false;

  }

  /*
   * The doBurger loop. After finishing the fries step, the robot will repeat the burger loop until we run out of time
   * It will do its own loop until it needs to do the burger plating step. The burger will then send communication wirelessly to 
   * the Screwdriver control ESP-32.
  */
  while (doBurger) {
    // Execute tasks 0 to 16
    for (int i = 0; i < 16; i++) {
      switch (i) {
        case 0: burgerTask0(); break;
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
        case 12: burgerTask12(); break;
        case 13: burgerTask13(); break;
        case 14: {
          burgerTask14();
          // After task 14, send message to server and wait for response
          // Message to server acts like a "I am here"
          // Response from server acts like "I have arrived"
          const char* instruction = "Reached Task 14";
          client.write(instruction, strlen(instruction));
          Serial.printf("Sent instruction: %s\n", instruction);

          // Wait for the command to execute task 15
          char command[50];
          while (true) {
            if (xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
              if (strcmp(command, "Execute Task 15") == 0) {
                break;
              }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as needed
          }
          // Execute task 15
          burgerTask15();
          instruction = "Finished Task 15";
          client.write(instruction, strlen(instruction));
          Serial.printf("Sent instruction: %s\n", instruction);

          //Wait for the server to confirm action
          while (!proceed) {
            delay(100);
          }
          delay(100);
          break;
        }
        case 15: burgerTask16(); break;
      }
    }
  }
}

// burger tasks from task 0 to task 15
void burgerTask0() { 
  
  Serial.println("Executing dummy task 0"); 
  delay(1000);
}

void burgerTask1() { 
  Serial.println("Executing dummy task 1"); 
  delay(1000);
}

void burgerTask2() { 
  Serial.println("Executing dummy task 2"); 
  delay(1000);
}

void burgerTask3() { 
  Serial.println("Executing dummy task 3"); 
  delay(1000);
}

void burgerTask4() { 
  Serial.println("Executing dummy task 4"); 
  delay(1000);
}

void burgerTask5() { 
  Serial.println("Executing dummy task 5"); 
  delay(1000);
}

void burgerTask6() { 
  Serial.println("Executing dummy task 6"); 
  delay(1000);
}

void burgerTask7() { 
  Serial.println("Executing dummy task 7"); 
  delay(1000);
}

void burgerTask8() { 
  Serial.println("Executing dummy task 8"); 
  delay(1000);
}

void burgerTask9() { 
  Serial.println("Executing dummy task 9"); 
  delay(1000);
}

void burgerTask10() { 
  Serial.println("Executing dummy task 10"); 
  delay(1000);
}

void burgerTask11() { 
  Serial.println("Executing dummy task 11"); 
  delay(1000);
}

void burgerTask12() { 
  Serial.println("Executing dummy task 12"); 
  delay(1000);
}

void burgerTask13() { 
  Serial.println("Executing dummy task 13"); 
  delay(1000);
}

void burgerTask14() { 
  Serial.println("Executing dummy task 14"); 
  delay(1000);
}

void burgerTask15() { 
  Serial.println("Executing dummy task 15"); 
  delay(1000);
}

void burgerTask16() { 
  Serial.println("Executing dummy task 16"); 
  delay(1000);
}

//fries tasks from task 0 to task 6
void friesTask0() { 
  Serial.println("Executing fries task 0"); 
  delay(1000);
}

void friesTask1() { 
  Serial.println("Executing fries task 1"); 
  delay(1000);
}

void friesTask2() { 
  Serial.println("Executing fries task 2"); 
  delay(1000);
}

void friesTask3() { 
  Serial.println("Executing fries task 3"); 
  delay(1000);
}

void friesTask4() { 
  Serial.println("Executing fries task 4"); 
  delay(1000);
}

void friesTask5() { 
  Serial.println("Executing fries task 5"); 
  delay(2000);
}

void friesTask6() { 
  Serial.println("Executing fries task 6"); 
  delay(1000);
}

void friesTask7() {
   Serial.println("Executing fries task 7"); 
   setCommPinOutput(7);
   delay(50);

   delay(1000);
   }


// Revised cubic easing function with slower easing-in
float easeInOutCubicSlow(float t) {
  t *= 2;
  if (t < 1) {
    return 0.5 * (t * t * t);
  }
  t -= 2;
  return 0.5 * (t * t * t + 2);
}

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
