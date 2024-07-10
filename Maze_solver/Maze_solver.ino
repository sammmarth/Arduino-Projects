// Define motor driver pins (using analog pins)
const int in1 = 8; 
const int in2 = 9;
const int in3 = 12;
const int in4 = 13;

// Define enable pins for PWM control
const int ena = 10;  // PWM pin for motor A
const int enb = 11; // PWM pin for motor B

// Define IR sensor pins
const int s1 = 2;  // Leftmost sensor
const int s2 = 3;  // Left sensor
const int s3 = 4;  // Middle sensor
const int s4 = 5;  // Right sensor
const int s5 = 6;  // Rightmost sensor

// Speed constants
const int Speed =      65; // Base speed for motors
const int turnSpeed =  55; // Speed during turns
const int slowSpeed =  50; // Speed for slow turns
const int uTurnSpeed = 50; // Speed for U-turns

// Read sensor values
int sensorValues[5];

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  // Set motor pins as outputs
  pinMode(in1, OUTPUT);   
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
   
  // Set enable pins as outputs
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  // Set IR sensor pins as inputs
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);

  // Initialize motor speed
  analogWrite(ena, Speed);
  analogWrite(enb, Speed);
}

void loop() {
  // Read sensor values
  sensorValues[0] = digitalRead(s1);
  sensorValues[1] = digitalRead(s2);
  sensorValues[2] = digitalRead(s3);
  sensorValues[3] = digitalRead(s4);
  sensorValues[4] = digitalRead(s5);

  // Print sensor values to Serial Monitor
  Serial.print("Sensor Values: ");
  for(int i = 0; i < 5; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Enhanced line following logic
  if (sensorValues[4] == HIGH && sensorValues[3] == HIGH && sensorValues[2] == LOW && sensorValues[1] == HIGH && sensorValues[0] == HIGH) {
    // Middle sensor on line, move forward
    Serial.println("Action: Move Forward");
    moveForward(Speed);
  } 
  else if (sensorValues[4] == HIGH && sensorValues[3] == HIGH && sensorValues[2] == LOW && sensorValues[1] == LOW && sensorValues[0] == LOW) {
    // Sharp left turn
    Serial.println("Action: Sharp Left");
    sharpLeft(turnSpeed);
  }
  else if (sensorValues[4] == LOW && sensorValues[3] == LOW && sensorValues[2] == LOW && sensorValues[1] == HIGH && sensorValues[0] == HIGH) {
    // Sharp right turn
    Serial.println("Action: Sharp Right");
    sharpRight(turnSpeed);
  }
  else if (sensorValues[4] == HIGH && sensorValues[3] == HIGH && sensorValues[2] == HIGH && sensorValues[1] == LOW && sensorValues[0] == LOW) {
    // Slight left turn
    Serial.println("Action: Slight Left");
    slightLeft(slowSpeed);
  }
  else if (sensorValues[4] == LOW && sensorValues[3] == LOW  && sensorValues[2] == HIGH && sensorValues[1] == HIGH && sensorValues[0] == HIGH) {
    // Slight right turn
    Serial.println("Action: Slight Right");
    slightRight(slowSpeed);
  }
  else if (sensorValues[4] == HIGH && sensorValues[3] == HIGH && sensorValues[2] == HIGH && sensorValues[1] == HIGH && sensorValues[0] == LOW) {
    // Slight left turn
    Serial.println("Action: Slight Left");
    slightLeft(slowSpeed);
  }
  else if (sensorValues[4] == LOW && sensorValues[3] == HIGH  && sensorValues[2] == HIGH && sensorValues[1] == HIGH && sensorValues[0] == HIGH) {
    // Slight right turn
    Serial.println("Action: Slight Right");
    slightRight(slowSpeed);
  }
  else if (sensorValues[4] == HIGH && sensorValues[3] == HIGH && sensorValues[2] == HIGH && sensorValues[1] == LOW && sensorValues[0] == HIGH) {
    // Slight left turn
    Serial.println("Action: Slight Left");
    slightLeft(slowSpeed);
  }
  else if (sensorValues[4] == HIGH && sensorValues[3] == LOW  && sensorValues[2] == HIGH && sensorValues[1] == HIGH && sensorValues[0] == HIGH) {
    // Slight right turn
    Serial.println("Action: Slight Right");
    slightRight(slowSpeed);
  }
  else if (sensorValues[0] == LOW && sensorValues[2] == LOW && sensorValues[3] == LOW && sensorValues[1] == LOW && sensorValues[4] == LOW) {
    // Potential crossroad or T-junction
    Serial.println("Action: T-Junction");
    sharpLeft(turnSpeed);
    handleTJunction();
  } else if (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && sensorValues[3] == HIGH && sensorValues[4] == HIGH) {
    // Dead end, perform U-turn
    Serial.println("Action: U-Turn");
    uTurn();

  } else {
    // Lost line, attempt to recover
    Serial.println("Action: Recover");
    recoverLine();
  }

  delay(50); // Add a small delay for readability
}

// Function to move forward
void moveForward(int Speed) {
  analogWrite(ena, Speed);
  analogWrite(enb, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void sharpRight(int Speed) {
  analogWrite(ena, Speed);
  analogWrite(enb, Speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void sharpLeft(int Speed) {
  analogWrite(ena, Speed);
  analogWrite(enb, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Function to turn slightly left
void slightLeft(int Speed) {
  analogWrite(ena, Speed);
  analogWrite(enb, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Function to turn slightly right
void slightRight(int Speed) {
  analogWrite(ena, Speed);
  analogWrite(enb, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Function to handle T-junction
void handleTJunction() {
  // Set motors to turn left
  analogWrite(ena, turnSpeed);
  analogWrite(enb, turnSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  delay(100); // Adjust the delay based on your robot's turning speed
  
  // After turning left, try to find the line again
  while (true) {
    int sensorValues[5];
    sensorValues[0] = digitalRead(s1);
    sensorValues[1] = digitalRead(s2);
    sensorValues[2] = digitalRead(s3);
    sensorValues[3] = digitalRead(s4);
    sensorValues[4] = digitalRead(s5);
    
    // Move forward if the middle sensor detects the line
    if (sensorValues[2] == LOW) {
      moveForward(Speed);
      break;
    }
    
    // Continue turning left if the line is not found yet
    sharpLeft(turnSpeed);
  }
}

// Function to perform a U-turn
void uTurn() {
  // Set motors to perform U-turn by turning left
  analogWrite(ena, uTurnSpeed);
  analogWrite(enb, uTurnSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  if (sensorValues[4] == HIGH && sensorValues[3] == HIGH && sensorValues[2] == LOW && sensorValues[1] == HIGH && sensorValues[0] == HIGH) {
  stopRobot();
  }
}

// Function to recover the line
void recoverLine() {
  // Set a time limit for recovery to prevent endless loop
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) { // Try to recover for 2 seconds
    int sensorValues[5];
    sensorValues[0] = digitalRead(s1);
    sensorValues[1] = digitalRead(s2);
    sensorValues[2] = digitalRead(s3);
    sensorValues[3] = digitalRead(s4);
    sensorValues[4] = digitalRead(s5);

    // Move forward if the middle sensor detects the line
    if (sensorValues[2] == LOW) {
      moveForward(Speed);
      return;
    }

    // Attempt to find the line by turning slightly left and right alternately
    slightLeft(turnSpeed);
    delay(100);
    slightRight(turnSpeed);
    delay(100);
  }

  // If the line is not found within the time limit, stop the robot
  stopRobot();
}

// Function to stop the robot
void stopRobot() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}