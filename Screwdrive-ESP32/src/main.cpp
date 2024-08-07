#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "U235-Control";  // Replace with your desired SSID
const char* password = "SkibidiToilet";  // Replace with your desired password

bool doBurgers = true;
bool ready = false;
bool proceed = false;
bool interProceed = false;
bool interReady = false;



#define commBit0 21
#define commBit1 22
#define commBit2 19
#define commBit3 18

AsyncServer server(80);  // Create a server object on port 80
AsyncClient* client = NULL;
QueueHandle_t responseQueue;

//WiFi connection methods
void TCP_Server(void* pvParameters);
void onClientConnect(void* arg, AsyncClient* newClient);

void ExecuteTasks(void *pvParameters);

//Task methods
void burgerTask1();
void burgerTask2();
void burgerTask3();
void burgerTask4();
void burgerTask5();
void burgerTask6();

//Wired Communication
void setCommPinOutput(int taskNumber);
int readCommPinInput();
bool wireCommunication(int taskNo);

void setup() {
  Serial.begin(9600);

  // Set up the ESP32 as an access point
  IPAddress IP(192, 168, 15, 221);  // Desired IP address
  IPAddress subnet(255, 255, 255, 0);  // Subnet mask
  WiFi.softAPConfig(IP, IP, subnet);
  WiFi.softAP(ssid, password);
  IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  WiFi.setSleep(false);

  // Create a queue to handle responses
  responseQueue = xQueueCreate(10, sizeof(char[50]));

  // Start the TCP server
  server.onClient(&onClientConnect, &server);
  server.begin();

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    TCP_Server,       // Task function
    "TCP Server",     // Name of the task (for debugging)
    4096,             // Stack size (bytes)
    NULL,             // Parameter to pass to the task
    1,                // Task priority
    NULL,             // Task handle
    0                 // Core to run the task on (0 in this case)
  );

  xTaskCreatePinnedToCore(
    ExecuteTasks,
    "Execute Tasks",
    4096,
    NULL,
    1,                // Task priority
    NULL,
    1                 // Core to run the task on (1 in this case)
  );
}

void loop() {
  // Main loop can run other tasks if needed
  vTaskDelete(NULL);
}

// Task for handling WiFi AP and client communication on core 0
void TCP_Server(void* pvParameters) {
  server.onClient([](void* arg, AsyncClient* newClient) {
    Serial.println("Client connected");

    if (client != NULL) {
      delete client;
    }
    client = newClient;

    client->onData([](void* arg, AsyncClient* client, void* data, size_t len) {
      char instruction[50];
      strncpy(instruction, (char*)data, len);
      instruction[len] = '\0';
      Serial.printf("Received instruction from client: %s\n", instruction);

      // Check for specific instructions and respond accordingly
      if (doBurgers && strcmp(instruction, "Reached Task 12") == 0) {
        const char* response = "Execute Task 13";
        while (!ready) {
          delay(100);
        }
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }

      if (doBurgers && strcmp(instruction, "Finished Task 13") == 0) {
        const char* response = "Action Received";
        proceed = true;
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }

      /*
      if (!doBurgers && strcmp(instruction, "Reached Fries Task 2") == 0) {
        const char* response = "Execute Task 3";
        while (!ready) {
          delay(100);
        }
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }

      if(!doBurgers && strcmp(instruction, "Finished Task 3") == 0) {
        const char* response = "Action Received";
        proceed = true;
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }
      */
    });

    client->onDisconnect([](void* arg, AsyncClient* c) {
      Serial.println("Client disconnected");
      if (client == c) {
        client = NULL;
      }
      delete c;
    }, NULL);
  }, NULL);

  server.begin();

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to handle server tasks
  }
}

void ExecuteTasks(void *pvParameters) {
  /*
  if (!doBurgers) {
    for (int i = 1; i < 7; i++) {
      switch (i) {

        case 1: friesTask1(); break;
        case 2: friesTask2(); break;
        case 3: friesTask3(); break;
        case 4: friesTask4(); break;
        case 5: {
          friesTask5();
          // After doing task 5, we wait for Screwdriver robot to show up so we can hand off the fries
          ready = true;

          // Wait for the command to execute task 6
          char command[50];
          while (!proceed) {
              delay(50);
            }
          vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as needed
          ready = false;
          proceed = false;
          // Execute task 6
          friesTask6();
          break;
        }
        case 6: friesTask7(); break;
      }
    }
    doBurgers = true;
  }
  */
  while (doBurgers) {
    // Execute tasks 0 to 16
    for (int i = 1; i < 7; i++) {
      switch (i) {
        case 1: burgerTask1(); break;
        case 2: burgerTask2(); break;
        case 3: burgerTask3(); break;
        case 4: {
          burgerTask4();
          // After task 14, send message to server and wait for response
          // Message to server acts like a "I am here"
          // Response from server acts like "I have arrived"
          ready = true;

          // Wait for the command to execute task 15
          char command[50];
          while (!proceed) {
            delay(50);
          }
          vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as needed
          // Execute task 15
          ready = false;
          proceed = false;
          burgerTask5();
          break;
        }
        case 5: burgerTask6(); break;
      }
    }
  }
}

// Callback when a client connects to the TCP server
void onClientConnect(void* arg, AsyncClient* newClient) {
  Serial.println("Client connected");

  if (client != NULL) {
    delete client;
  }
  client = newClient;

  client->onData([](void* arg, AsyncClient* client, void* data, size_t len) {
    char command[50];
    strncpy(command, (char*)data, len);
    command[len] = '\0';
    Serial.printf("Received command from client: %s\n", command);
    xQueueSend(responseQueue, &command, portMAX_DELAY);
  });

  client->onDisconnect([](void* arg, AsyncClient* c) {
    Serial.println("Client disconnected");
    if (client == c) {
      client = NULL;
    }
    delete c;
  }, NULL);
}

void burgerTask1() { 
  Serial.println("Executing burger task 1");
  
  Serial.println("Completed burger task 1");
}

void burgerTask2() { 
  Serial.println("Executing burger task 2"); 
  delay(1000);
  Serial.println("Completed burger task 2");
}

void burgerTask3() { 
  Serial.println("Executing burger task 3"); 
  delay(1000); 
  Serial.println("Completed burger task 3");
}

void burgerTask4() { 
  Serial.println("Executing burger task 4"); 
  delay(1000); 
  Serial.println("Completed burger task 4");
}

void burgerTask5() { 
  Serial.println("Executing burger task 5"); 
  delay(1000); 
  Serial.println("Completed burger task 5");
}

void burgerTask6() { 
  Serial.println("Executing burger task 6"); 
  delay(1000); 
  Serial.println("Completed burger task 6");
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
  int armAngle;
  int turnAngle;
  switch (taskNo)
  {
  case 1: {
    armAngle  = flatSurface;
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

  case 7:{
    armAngle = flatSurface;
    turnAngle = centre;
  }

  default:
    break;
  }

  setCommPinOutput(taskNo);
  Serial.print("Task Number Sent: ");
  Serial.println(taskNo);

  digitalWrite(ready, HIGH);  // Signal ESP-2 that data is ready
  delay(100);
  if(armAngle != currentArmServoPos){
    smoothServoControl(armAngle, 1);
  }
  if (turnAngle != currentTurnServoPos){
    smoothServoControl(turnAngle, 2); 
    delay(100); //some delay for turning
  }
  waitForSignal(signal);  // Wait for ESP-2 to complete the task
  digitalWrite(ready, LOW);  // Reset the ready signal
  // After receiving the signal from ESP-2, reset the output
  setCommPinOutput(0);
  Serial.println("Received completion signal from ESP-2");
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


