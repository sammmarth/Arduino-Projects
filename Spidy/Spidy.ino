#include <Servo.h>

// Define the servos
Servo frontLeftCornerServo;
Servo frontRightCornerServo;
Servo backLeftCornerServo;
Servo backRightCornerServo;

Servo frontLeftLegServo;
Servo frontRightLegServo;
Servo backLeftLegServo;
Servo backRightLegServo;

Servo headServo;

// Define the pins for the servos
const int frontLeftCornerServoPin = 2;
const int frontRightCornerServoPin = 3;
const int backLeftCornerServoPin = 4;
const int backRightCornerServoPin = 5;

const int frontLeftLegServoPin = 6;
const int frontRightLegServoPin = 7;
const int backLeftLegServoPin = 8;
const int backRightLegServoPin = 9;

const int headServoPin = 10;

// Define ultrasonic sensor pins
const int trigPin = 11;
const int echoPin = 12;

// Define distance threshold for obstacle detection
const int distanceThreshold = 20; // in cm

void setup() {
  Serial.begin(9600);

  // Attach corner servos to pins
  frontLeftCornerServo.attach(frontLeftCornerServoPin);
  frontRightCornerServo.attach(frontRightCornerServoPin);
  backLeftCornerServo.attach(backLeftCornerServoPin);
  backRightCornerServo.attach(backRightCornerServoPin);
  
  // Attach leg servos to pins
  frontLeftLegServo.attach(frontLeftLegServoPin);
  frontRightLegServo.attach(frontRightLegServoPin);
  backLeftLegServo.attach(backLeftLegServoPin);
  backRightLegServo.attach(backRightLegServoPin);
  
  // Attach head servo to pin
  headServo.attach(headServoPin);

  // Set up the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize servos to starting positions
  initializeServos();
}

void loop() {
  // Check for obstacles
  int distance = getDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < distanceThreshold) {
    // Obstacle detected, perform avoidance behavior
    avoidObstacle();
  } else {
    // No obstacle, continue normal walking
    walk();
  }
}

void initializeServos() {
  // Initialize all servos to default positions
  frontLeftCornerServo.write(90); // Assume 90 is the neutral position
  frontRightCornerServo.write(90); 
  backLeftCornerServo.write(90);
  backRightCornerServo.write(90);
  
  frontLeftLegServo.write(90);
  frontRightLegServo.write(90); 
  backLeftLegServo.write(90);
  backRightLegServo.write(90);
  
  headServo.write(90); // Center the head
}

int getDistance() {
  // Scan left
  headServo.write(45);
  delay(500);
  int distanceLeft = measureDistance();
  Serial.print("Distance Left: ");
  Serial.println(distanceLeft);

  // Scan right
  headServo.write(135);
  delay(500);
  int distanceRight = measureDistance();
  Serial.print("Distance Right: ");
  Serial.println(distanceRight);

  // Center the head
  headServo.write(90);

  // Return the smaller distance
  return min(distanceLeft, distanceRight);
}

int measureDistance() {
  // Send a pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the pulse duration from the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  int distance = duration * 0.034 / 2;

  return distance;
}

void walk() {
  Serial.println("Walking...");
  
  // Basic walking gait - alternate leg movement
  frontLeftLegServo.write(45);  // Move front left leg forward
  frontRightLegServo.write(135); // Move front right leg backward
  backLeftLegServo.write(45);    // Move back left leg forward
  backRightLegServo.write(135);  // Move back right leg backward

  frontLeftCornerServo.write(45); // Move front left corner
  frontRightCornerServo.write(135); // Move front right corner
  backLeftCornerServo.write(45); // Move back left corner
  backRightCornerServo.write(135); // Move back right corner
  
  delay(200);
  
  frontLeftLegServo.write(90);  // Return front left leg to neutral
  frontRightLegServo.write(90); // Return front right leg to neutral
  backLeftLegServo.write(90);   // Return back left leg to neutral
  backRightLegServo.write(90);  // Return back right leg to neutral

  frontLeftCornerServo.write(90); // Return front left corner to neutral
  frontRightCornerServo.write(90); // Return front right corner to neutral
  backLeftCornerServo.write(90); // Return back left corner to neutral
  backRightCornerServo.write(90); // Return back right corner to neutral

  delay(200);
}

void avoidObstacle() {
  Serial.println("Avoiding obstacle...");
  
  // Basic obstacle avoidance - turn to the right
  frontLeftLegServo.write(135);  // Move left legs backward
  backLeftLegServo.write(135);   // Move left legs backward
  
  frontRightLegServo.write(45);  // Move right legs forward
  backRightLegServo.write(45);   // Move right legs forward

  frontLeftCornerServo.write(135); // Move left corners backward
  backLeftCornerServo.write(135); // Move left corners backward
  
  frontRightCornerServo.write(45); // Move right corners forward
  backRightCornerServo.write(45); // Move right corners forward

  delay(200);
  
  frontLeftLegServo.write(90);  // Return left legs to neutral
  backLeftLegServo.write(90);   // Return left legs to neutral
  
  frontRightLegServo.write(90); // Return right legs to neutral
  backRightLegServo.write(90);  // Return right legs to neutral

  frontLeftCornerServo.write(90); // Return left corners to neutral
  backLeftCornerServo.write(90); // Return left corners to neutral
  
  frontRightCornerServo.write(90); // Return right corners to neutral
  backRightCornerServo.write(90); // Return right corners to neutral

  delay(200);
}
