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

void friesTask1();
void friesTask2();
void friesTask3();
void friesTask4();
void friesTask5();
void friesTask6();
void friesTask7();

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

void burgerTask7() { 
  Serial.println("Executing burger task 7"); 
  delay(1000); 
  Serial.println("Completed burger task 7");
}

void burgerTask8() { 
  Serial.println("Executing burger task 8"); 
  delay(1000); 
  Serial.println("Completed burger task 8");
}

void burgerTask9() { 
  Serial.println("Executing burger task 9"); 
  delay(1000); 
  Serial.println("Completed burger task 9");
}

void burgerTask10() { 
  Serial.println("Executing burger task 10"); 
  delay(1000); 
  Serial.println("Completed burger task 10");
}

void burgerTask11() { 
  Serial.println("Executing burger task 11"); 
  delay(1000); 
  Serial.println("Completed burger task 11");
}

void burgerTask12() { 
  Serial.println("Executing burger task 12"); 
  delay(1000); 
  Serial.println("Completed burger task 12");
}

void burgerTask13() { 
  Serial.println("Executing burger task 13"); 
  delay(1000); 
  Serial.println("Completed burger task 13");
}

void burgerTask14() { 
  Serial.println("Executing burger task 14"); 
  delay(1000); 
  Serial.println("Completed burger task 14");
}

void burgerTask15() { 
  Serial.println("Executing burger task 15"); 
  delay(1000); 
  Serial.println("Completed burger task 15");
}

void friesTask0() { 
  Serial.println("Executing fries task 0"); 
  delay(1000); 
  Serial.println("Completed fries task 0");
}

void friesTask1() { 
  Serial.println("Executing fries task 1"); 
  delay(1000); 
  Serial.println("Completed fries task 1");
}

void friesTask2() { 
  Serial.println("Executing fries task 2"); 
  delay(1000); 
  Serial.println("Completed fries task 2");
}

void friesTask3() { 
  Serial.println("Executing fries task 3"); 
  delay(1000); 
  Serial.println("Completed fries task 3");
}

void friesTask4() { 
  Serial.println("Executing fries task 4"); 
  delay(1000); 
  Serial.println("Completed fries task 4");
}

void friesTask5() { 
  Serial.println("Executing fries task 5"); 
  delay(2000); 
  Serial.println("Completed fries task 5");
}

void friesTask6() { 
  Serial.println("Executing fries task 6"); 
  delay(1000); 
  Serial.println("Completed fries task 6");
}

void friesTask7() { 
  Serial.println("Executing fries task 7"); 
  delay(1000); 
  Serial.println("Completed fries task 7");
}


bool wireCommunication(int taskNo){
  int recievedNo = 0;
  setCommPinOutput(taskNo);
  recievedNo == readCommPinInput();
  if(recievedNo == taskNo){
    return true;
  }
  return false;
}



/* @brief Converts integer into binary
 *
 * This method converts an integer value into a binary value to be send down communication pins
 * 
 * @param integerValue the integer value to be converted
*/
void setCommPinOutput(int integerValue){
  pinMode(commBit0, OUTPUT);
  pinMode(commBit1, OUTPUT);
  pinMode(commBit2, OUTPUT);
  pinMode(commBit3, OUTPUT);
  switch(integerValue){
    case 0: 
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, LOW);
      break;

    case 1:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, LOW);
      break;

    case 2:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, LOW);
      break;

    case 3:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, LOW);
      break;
    
    case 4:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, LOW);
      break;
    
    case 5:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, LOW);
      break;

    case 6:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, LOW);
      break;

    case 7:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, LOW);
      break;
    
    case 8:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, HIGH);
      break;
    
    case 9:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, HIGH);
      break;

    case 10:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, HIGH);
      break;

    case 11:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, LOW);
      digitalWrite(commBit3, HIGH);
      break;

    case 12:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, HIGH);
      break;

    case 13:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, LOW);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, HIGH);
      break;

    case 14:
      digitalWrite(commBit0, LOW);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, HIGH);
      break;

    case 15:
      digitalWrite(commBit0, HIGH);
      digitalWrite(commBit1, HIGH);
      digitalWrite(commBit2, HIGH);
      digitalWrite(commBit3, HIGH);
      break;
    default:
      break;
  }
}



/*
 * @brief Reads pin values and converts to integer
 *
 * The method reads the communication pins and converts the binary signal into integer value
 * 
*/
int readCommPinInput(){

  pinMode(commBit0, INPUT);
  pinMode(commBit1, INPUT);
  pinMode(commBit2, INPUT);
  pinMode(commBit3, INPUT);

  if (commBit0 == LOW && commBit1 == LOW && commBit2 == LOW && commBit3 == LOW) {
    return 0;
  }
  else if (commBit0 == HIGH && commBit1 == LOW && commBit2 == LOW && commBit3 == LOW) {
    return 1;
  }
  else if (commBit0 == LOW && commBit1 == HIGH && commBit2 == LOW && commBit3 == LOW) {
    return 2;
  }
  else if (commBit0 == HIGH && commBit1 == HIGH && commBit2 == LOW && commBit3 == LOW) {
    return 3;
  }
  else if (commBit0 == LOW && commBit1 == LOW && commBit2 == HIGH && commBit3 == LOW) {
    return 4;
  }
  else if (commBit0 == HIGH && commBit1 == LOW && commBit2 == HIGH && commBit3 == LOW) {
    return 5;
  }
  else if (commBit0 == LOW && commBit1 == HIGH && commBit2 == HIGH && commBit3 == LOW) {
    return 6;
  }
  else if (commBit0 == HIGH && commBit1 == HIGH && commBit2 == HIGH && commBit3 == LOW) {
    return 7;
  }
  else if (commBit0 == LOW && commBit1 == LOW && commBit2 == LOW && commBit3 == HIGH) {
    return 8;
  }
  else if (commBit0 == HIGH && commBit1 == LOW && commBit2 == LOW && commBit3 == HIGH) {
    return 9;
  }
  else if (commBit0 == LOW && commBit1 == HIGH && commBit2 == LOW && commBit3 == HIGH) {
    return 10;
  }
  else if (commBit0 == HIGH && commBit1 == HIGH && commBit2 == LOW && commBit3 == HIGH) {
    return 11;
  }
  else if (commBit0 == LOW && commBit1 == LOW && commBit2 == HIGH && commBit3 == HIGH) {
    return 12;
  }
  else if (commBit0 == HIGH && commBit1 == LOW && commBit2 == HIGH && commBit3 == HIGH) {
    return 13;
  }
  else if (commBit0 == LOW && commBit1 == HIGH && commBit2 == HIGH && commBit3 == HIGH) {
    return 14;
  }
  else if (commBit0 == HIGH && commBit1 == HIGH && commBit2 == HIGH && commBit3 == HIGH) {
    return 15;
  }
  else{
    return -1; //something went very very wrong here if it returns -1
  }
}
  

