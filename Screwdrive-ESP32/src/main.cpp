#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "U235-Control";  // Replace with your desired SSID
const char* password = "SkibidiToilet";  // Replace with your desired password

bool doBurgers = false;
volatile bool ready = false;
volatile bool proceed = false;

AsyncServer server(80);  // Create a server object on port 80
AsyncClient* client = NULL;
QueueHandle_t responseQueue;

void TCP_Server(void* pvParameters);
void onClientConnect(void* arg, AsyncClient* newClient);
void ExecuteTasks(void *pvParameters);
void burgerTask0();
void burgerTask1();
void burgerTask2();
void burgerTask3();
void burgerTask4();
void burgerTask5();
void burgerTask6();
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
      if (doBurgers && strcmp(instruction, "Reached Task 14") == 0) {
        const char* response = "Execute Task 15";
        while (!ready) {
          delay(100);
        }
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }

      if (doBurgers && strcmp(instruction, "Finished Task 15") == 0) {
        const char* response = "Action Received";
        proceed = true;
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }

      if (!doBurgers && strcmp(instruction, "Reached Fries Task 5") == 0) {
        const char* response = "Execute Task 6";
        while (!ready) {
          delay(100);
        }
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }

      if(!doBurgers && strcmp(instruction, "Finished Task 6") == 0) {
        const char* response = "Action Received";
        proceed = true;
        client->write(response, strlen(response));
        Serial.printf("Sent response: %s\n", response);
      }
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
  if (!doBurgers) {
    for (int i = 0; i < 7; i++) {
      switch (i) {
        case 0: friesTask0(); break;
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

  while (doBurgers) {
    // Execute tasks 0 to 16
    for (int i = 0; i < 7; i++) {
      switch (i) {
        case 0: burgerTask0(); break;
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

// burger tasks from task 0 to task 15
void burgerTask0() { Serial.println("Executing burger task 0"); delay(1000); }
void burgerTask1() { Serial.println("Executing burger task 1"); delay(1000); }
void burgerTask2() { Serial.println("Executing burger task 2"); delay(1000); }
void burgerTask3() { Serial.println("Executing burger task 3"); delay(1000); }
void burgerTask4() { Serial.println("Executing burger task 4"); delay(1000); }
void burgerTask5() { Serial.println("Executing burger task 5"); delay(1000); }
void burgerTask6() { Serial.println("Executing burger task 6"); delay(1000); }

// fries tasks from task 0 to task 6
void friesTask0() { Serial.println("Executing fries task 0"); delay(1000); }
void friesTask1() { Serial.println("Executing fries task 1"); delay(1000); }
void friesTask2() { Serial.println("Executing fries task 2"); delay(1000); }
void friesTask3() { Serial.println("Executing fries task 3"); delay(1000); }
void friesTask4() { Serial.println("Executing fries task 4"); delay(1000); }
void friesTask5() { Serial.println("Executing fries task 5"); delay(2000); }
void friesTask6() { Serial.println("Executing fries task 6"); delay(1000); }
void friesTask7() { Serial.println("Executing fries task 7"); delay(1000); }
