#include <Arduino.h>

#define MOTOR_PIN1 2  // GPIO pin to control motor input 1
#define MOTOR_PIN2 4  // GPIO pin to control motor input 2
#define ENCODER_CLK 25  // GPIO pin connected to encoder A (CLK)
#define ENCODER_DT  26  // GPIO pin connected to encoder B (DT)

volatile int encoderCount = 0;  // Counter for encoder clicks
int lastCLKState;

void startMotor(bool forward) {
  if (forward) {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
  } else {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
  }
}

void stopMotor() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
}


void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);

  // Read the initial state of the encoder
  lastCLKState = digitalRead(ENCODER_CLK);

  // Start the motor
  startMotor(true);
}

void loop() {
  long time = millis();
  while (millis() - time < 1100)
  {
    startMotor(true);
    Serial.println("Running...");
  }
  stopMotor();
  Serial.println("Stop");
  delay(5000);
  
  time = millis();
  while (millis() - time < 1100){
    startMotor(false);
    Serial.println("Running...");
  }
  stopMotor();
  Serial.println("Stop");
  delay(5000);
}

