#include "Arduino.h"
#include "WiFi.h"
#include "AsyncTCP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

const char* ssid = "U235-Control";  // Replace with your AP's SSID
const char* password = "SkibidiToilet";  // Replace with your AP's password
const char* server_ip = "192.168.15.221";  // Default IP for ESP32 AP
const uint16_t server_port = 80;  // Replace with your server's port
bool doBurger = false;
volatile bool proceed = false;

#define commBit0 23
#define commBit1 22
#define commBit2 1

AsyncClient client;
QueueHandle_t commandQueue;

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

void setup() {
  Serial.begin(9600);

  // Connect to ESP32 AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to ESP32 AP...");
  }
  Serial.println("Connected to ESP32 AP");

  // Create FreeRTOS queue
  commandQueue = xQueueCreate(10, sizeof(char) * 50);

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    TCP_Client,       // Task function
    "TCP Client",     // Name of the task (for debugging)
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

// Task to execute tasks 0-15 on core 1
void ExecuteTasks(void *pvParameters) {
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
void friesTask0() { Serial.println("Executing fries task 0"); delay(1000);}
void friesTask1() { Serial.println("Executing fries task 1"); delay(1000);}
void friesTask2() { Serial.println("Executing fries task 2"); delay(1000);}
void friesTask3() { Serial.println("Executing fries task 3"); delay(1000);}
void friesTask4() { Serial.println("Executing fries task 4"); delay(1000);}
void friesTask5() { Serial.println("Executing fries task 5"); delay(2000);}
void friesTask6() { Serial.println("Executing fries task 6"); delay(1000);}


void friesTask7() {
   Serial.println("Executing fries task 7"); 
   pinMode(commBit0, OUTPUT);
   pinMode(commBit1, OUTPUT);
   pinMode(commBit2, OUTPUT);
   digitalWrite(commBit0, HIGH);
   digitalWrite(commBit1, HIGH);
   digitalWrite(commBit2, HIGH);
   pinMode(commBit0, INPUT);
   pinMode(commBit1, INPUT);
   pinMode(commBit2, INPUT);
   delay(1000);
   }



void setCommPinOutput(int taskNumber){
  pinMode(commBit0, OUTPUT);
  pinMode(commBit1, OUTPUT);
  pinMode(commBit2, OUTPUT);
  switch(taskNumber){
    case 0: 
    digitalWrite(commBit0, HIGH);
    digitalWrite(commBit1, HIGH);
    digitalWrite(commBit2, HIGH);
  }

  
   
}
