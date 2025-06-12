#include <SoftwareSerial.h>

// HC Serial
const int BLUETOOTH_RX_PIN = 11;
const int BLUETOOTH_TX_PIN = 12;
SoftwareSerial HCSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN);

//==================================================================
// HARDWARE CONFIGURATION & PIN DEFINITIONS
//==================================================================

// --- L298N Motor Driver ---
const int L298N_IN1_PIN = 2;
const int L298N_IN2_PIN = 3;
const int L298N_ENA = 9;  // ENA

const int L298N_IN3_PIN = 4;
const int L298N_IN4_PIN = 5;
const int L298N_ENB = 10;  // ENB

// --- Ultrasonic Distance Sensor (HY-SRF05) ---
const int ULTRASONIC_TRIG_PIN = 6;
const int ULTRASONIC_ECHO_PIN = 7;

// --- Piezo Buzzer ---
const int PIEZO_SPEAKER_PIN = 8;

//==================================================================
// OPERATING PARAMETERS
//==================================================================

int MAX_SPEED = 250;                         // Robot's travel speed (0-255)
const int TURN_SPEED_DIVISOR = 10;           // Used for arc turns. Higher value = sharper turn.
const int OBSTACLE_DETECTION_DISTANCE_STOP = 20; // Alert and stop car
const int OBSTACLE_DETECTION_DISTANCE = 30;  // Alert distance in cm
bool isStop = false;

//==================================================================
// SETUP FUNCTION
//==================================================================
void setup() {
  Serial.begin(9600);
  HCSerial.begin(9600);

  // Configure L298N pins
  pinMode(L298N_IN1_PIN, OUTPUT);
  pinMode(L298N_IN2_PIN, OUTPUT);
  pinMode(L298N_IN3_PIN, OUTPUT);
  pinMode(L298N_IN4_PIN, OUTPUT);
  pinMode(L298N_ENA, OUTPUT);
  pinMode(L298N_ENB, OUTPUT);

  // Configure Ultrasonic Sensor pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // Configure Piezo Buzzer pin (Active-Low)
  pinMode(PIEZO_SPEAKER_PIN, OUTPUT);
  digitalWrite(PIEZO_SPEAKER_PIN, HIGH);

  Serial.println("System initialized. Awaiting commands.");
  delay(1000);  // Wait for sensors to stabilize
}

//==================================================================
// MOTOR CONTROL FUNCTIONS
//==================================================================

void stopMotors() {
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, LOW);
  analogWrite(L298N_ENA, 0);
  analogWrite(L298N_ENB, 0);
}

void moveForward() {
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);
  digitalWrite(L298N_IN3_PIN, HIGH);
  digitalWrite(L298N_IN4_PIN, LOW);
  analogWrite(L298N_ENA, MAX_SPEED);
  analogWrite(L298N_ENB, MAX_SPEED);
}

void moveBackward() {
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, HIGH);
  analogWrite(L298N_ENA, MAX_SPEED);
  analogWrite(L298N_ENB, MAX_SPEED);
}

void turnLeftOnSpot() {
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, HIGH);
  digitalWrite(L298N_IN4_PIN, LOW);
  analogWrite(L298N_ENA, MAX_SPEED);
  analogWrite(L298N_ENB, MAX_SPEED);
}

void turnRightOnSpot() {
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, HIGH);
  analogWrite(L298N_ENA, MAX_SPEED);
  analogWrite(L298N_ENB, MAX_SPEED);
}

void arcForwardLeft() {
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, HIGH);
  digitalWrite(L298N_IN4_PIN, LOW);
  analogWrite(L298N_ENA, MAX_SPEED / TURN_SPEED_DIVISOR);
  analogWrite(L298N_ENB, MAX_SPEED);
}

void arcForwardRight() {
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, HIGH);
  analogWrite(L298N_ENA, MAX_SPEED);
  analogWrite(L298N_ENB, MAX_SPEED / TURN_SPEED_DIVISOR);
}

void arcBackwardLeft() {
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, HIGH);
  digitalWrite(L298N_IN4_PIN, LOW);
  analogWrite(L298N_ENA, MAX_SPEED);
  analogWrite(L298N_ENB, MAX_SPEED / TURN_SPEED_DIVISOR);
}

void arcBackwardRight() {
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, HIGH);
  analogWrite(L298N_ENA, MAX_SPEED / TURN_SPEED_DIVISOR);
  analogWrite(L298N_ENB, MAX_SPEED);
}

//==================================================================
// SENSOR AND ALERT FUNCTIONS
//==================================================================

int getDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  
  if (duration > 0) {
    return duration / 29.412 / 2;
  }
  return 999;
}

void alertOnObstacle() {
  int distance = getDistance();

  if (distance < OBSTACLE_DETECTION_DISTANCE_STOP) {
    digitalWrite(PIEZO_SPEAKER_PIN, LOW);  // Turn buzzer ON
    isStop = true;
  } else if (distance < OBSTACLE_DETECTION_DISTANCE) {
    digitalWrite(PIEZO_SPEAKER_PIN, LOW);  // Turn buzzer ON
    isStop = false;
  } else {
    digitalWrite(PIEZO_SPEAKER_PIN, HIGH);  // Turn buzzer OFF
    isStop = false;
  }
}

//==================================================================
// MAIN LOOP
//==================================================================

char command = 'S';

void loop() {

  // Check for an incoming command from the Serial Monitor
  if (HCSerial.available()) {
    command = HCSerial.read();
  }

  alertOnObstacle();

  // Execute an action immediately based on the command character
  switch (command) {
    case 'F': moveForward(); break;       // Forward
    case 'B': 
      if (isStop) stopMotors();
      else moveBackward(); 
      break;      // Backward
    case 'L': turnLeftOnSpot(); break;    // Turn Left on the spot
    case 'R': turnRightOnSpot(); break;   // Turn Right on the spot
    case 'G': arcForwardLeft(); break;    // Arc Forward-Left
    case 'I': arcForwardRight(); break;   // Arc Forward-Right
    case 'H': 
      if (isStop) stopMotors();
      else arcBackwardLeft(); 
      break;   // Arc Backward-Left
    case 'J':
      if (isStop) stopMotors(); 
      else arcBackwardRight(); 
      break;  // Arc Backward-Right
    case 'S':                             // Stop command
    default:                              // Any other character also stops the robot
      stopMotors();
      break;
  }
}