#include <SoftwareSerial.h>

// Define pins for ultrasonic sensors
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_LEFT 4
#define ECHO_LEFT 5
#define TRIG_RIGHT 6
#define ECHO_RIGHT 7

// Define pins for motor driver
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

const int ena = A1;  // PWM pin for motor A
const int enb = A2; // PWM pin for motor B

const int Speed = 150; // Adjusted speed value (range 0-255)

// Define pin for buzzer (horn)
#define BUZZER 13
#define EMERGENCY_BRAKE A0
// Define pin for mode button
#define MODE_BUTTON 12

// Bluetooth module pins
SoftwareSerial BTSerial(0, 1); // RX | TX

// Threshold distance in centimeters
const int threshold = 11;

// Variables for path retribution
bool deviatingLeft = false;
bool deviatingRight = false;
unsigned long deviationStartTime = 0;
const unsigned long deviationDuration = 2000; // Time to deviate in milliseconds

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    BTSerial.begin(9600);

    // Set up the ultrasonic sensor pins
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);

    // Set up the motor driver pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Set up the buzzer pin
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    // Set up the mode button pin
    pinMode(MODE_BUTTON, INPUT_PULLUP);

    // Set up the emergency brake pin
    pinMode(EMERGENCY_BRAKE, INPUT_PULLUP);

    // Set PWM pins
    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);
}

void loop() {
    // Check the mode button state
    bool isBluetoothMode = digitalRead(MODE_BUTTON) == LOW;

    // Check the emergency brake state
    if (digitalRead(EMERGENCY_BRAKE) == HIGH) {
        // Emergency brake activated
        stop();
        digitalWrite(BUZZER, HIGH);
        delay(100); // Horn duration
        digitalWrite(BUZZER, LOW);
        delay(100); // Pause before the next loop iteration
        return; // Skip the rest of the loop
    }

    if (isBluetoothMode) {
        // Bluetooth control mode
        stop();
        if (BTSerial.available()) {
            char command = BTSerial.read();
            handleBluetoothCommand(command);
        }
    } else {
        // Automated control mode using ultrasonic sensors
        automatedControl();
    }
}

void automatedControl() {
    // Read distances from ultrasonic sensors
    long distanceFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    long distanceLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    long distanceRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

    // Print distances to Serial Monitor
    Serial.print("Front: ");
    Serial.print(distanceFront);
    Serial.print(" cm, Left: ");
    Serial.print(distanceLeft);
    Serial.print(" cm, Right: ");
    Serial.print(distanceRight);
    Serial.println(" cm");

    // Obstacle avoidance and path retribution
    if (distanceFront < threshold) {
        // Obstacle detected in front, check sides
        if (distanceLeft < threshold && distanceRight < threshold) {
            // Obstacles on all sides, stop
            stop();
        } else if (distanceLeft > distanceRight) {
            // More space on the left, turn left
            turnLeft();
            deviatingLeft = true;
            deviatingRight = false;
            deviationStartTime = millis();
        } else if (distanceLeft < distanceRight) {
            // More space on the right, turn right
            turnRight();
            deviatingLeft = false;
            deviatingRight = true;
            deviationStartTime = millis();
        } else {
            // Equal space on left and right, turn left
            turnLeft();
            deviatingLeft = true;
            deviatingRight = false;
            deviationStartTime = millis();
        }
    } else {
        // No obstacle in front
        if (deviatingLeft || deviatingRight) {
            // Path retribution logic
            if (millis() - deviationStartTime < deviationDuration) {
                if (deviatingLeft) {
                    turnRight();
                } else if (deviatingRight) {
                    turnLeft();
                }
            } else {
                // Retribution completed
                deviatingLeft = false;
                deviatingRight = false;
                moveForward();
            }
        } else {
            // Move forward if not deviating
            moveForward();
        }
    }
}

long getDistance(int trigPin, int echoPin) {
    // Send a 10us pulse to trigger the sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pin and calculate distance
    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration / 2) / 29.1; // Convert duration to centimeters
    return distance;
}

void moveForward() {
    Serial.println("Forward");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ena, Speed);
    analogWrite(enb, Speed);
}

void turnLeft() {
    Serial.println("Left");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ena, Speed);
    analogWrite(enb, Speed);
}

void turnRight() {
    Serial.println("Right");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ena, Speed);
    analogWrite(enb, Speed);
}

void stop() {
    Serial.println("Stop");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ena, 0);
    analogWrite(enb, 0);
}

void handleBluetoothCommand(char command) {
    digitalRead(command);
        if(command =='F'){ // Forward
            moveForward();
            Serial.print("Going forward ");            
        }


        
        else if(command =='L'){ // Left
            turnLeft();            
            Serial.print("\nGoing left");
          }
        else if(command =='R'){ // Right
            turnRight();
            Serial.print("\nGoing right");
            }
        else if(command =='S'){ // Stop
            stop();
            Serial.print("\nStopping");
            }
        else if(command =='V'){// Horn
            digitalWrite(BUZZER, HIGH);
            delay(100); // Horn duration
            digitalWrite(BUZZER, LOW);
            Serial.print("\nBuzzer buzzed");
            }
        else{
            Serial.print("\nGive command");
        }
    }