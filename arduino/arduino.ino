#include <SoftwareSerial.h>

/*
  Using SoftwareSerial to simulate the second Serial
  
  Note:
    RX = 8
    TX = 9
*/
SoftwareSerial SimulatedSerial(8, 9);

// Using L298N to control the electricity
const int L298N_IN1_PIN = 2;
const int L298N_IN2_PIN = 3;
const int L298N_IN3_PIN = 4;
const int L298N_IN4_PIN = 5;
const int L298N_EN_PIN = 12;

// Using Ultra Sonic to calculate the distance between the car to object
const int HY_SRF05_TRIG_PIN = 6;
const int HY_SRF05_ECHO_PIN = 7;

// Using Piezo Speaker to alert when having object within distance
const int PIEZO_SPEAKER_PIN = 13;

// Speed will be from 0 to 255
const int SPEED = 255;

/*
  Direction of the car

  Note:
    F: Go Straight
    B: Go Back
    L: Go Left
    R: Go Right
    S: Pause (Stop)

  Default: 'S'
*/
char direction = 'F';

/* Initializes the setting up */
void setup() {

  // Initialize Serial Monitor's port
  Serial.begin(9600);
  SimulatedSerial.begin(115200);

  /*
    Initialize L298N PIN

    According the theory:
      The electricity will be from DC+ to DC-

    Note:
      If PIN is LOW, that means it's having an Anode (DC-)
      If PIN is HIGH, that means it's having a Cathode (DC+)
  */
  pinMode(L298N_IN1_PIN, OUTPUT);
  pinMode(L298N_IN2_PIN, OUTPUT);
  pinMode(L298N_IN3_PIN, OUTPUT);
  pinMode(L298N_IN4_PIN, OUTPUT);
  pinMode(L298N_EN_PIN, OUTPUT);

  // Initialize Ultra Sonic PIN
  pinMode(HY_SRF05_TRIG_PIN, OUTPUT);
  pinMode(HY_SRF05_ECHO_PIN, INPUT);

  // Initialize Piezo Speaker PIN
  pinMode(PIEZO_SPEAKER_PIN, OUTPUT);
}

/* Control 4 wheels to go UP */
void goStraight(int speed) {
  // The electricity will be from L298N_IN2_PIN to L298N_IN1_PIN to active gearmotor 1 and 2
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);

  // The electricity will be from L298N_IN4_PIN to L298N_IN3_PIN to active gearmotor 3 and 4
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, HIGH);

  // Speed of car
  analogWrite(L298N_EN_PIN, speed);
}

/* Control 4 wheels to go BACK */
void goBack(int speed) {
  // The electricity will be from L298N_IN1_PIN to L298N_IN2_PIN to active gearmotor 1 and 2
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);

  // The electricity will be from L298N_IN3_PIN to L298N_IN4_PIN to active gearmotor 3 and 4
  digitalWrite(L298N_IN3_PIN, HIGH);
  digitalWrite(L298N_IN4_PIN, LOW);

  // Speed of car
  analogWrite(L298N_EN_PIN, speed);
}

/* Control 2 wheels AT THE RIGHT to go LEFT */
void goLeft(int speed) {
  // The electricity of both L298N_IN1_PIN and L298N_IN2_PIN will be LOW to stop to go left
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);

  // Speed of car
  analogWrite(L298N_EN_PIN, speed);
}

/* Control 2 wheels AT THE LEFT to go RIGHT */
void goRight(int speed) {
  // The electricity will be from L298N_IN2_PIN to L298N_IN1_PIN to active gearmotor 1 and 2
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);

  // Speed of car
  analogWrite(L298N_EN_PIN, speed);
}


/* Control 4 wheels to stop */
void stop() {
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, LOW);
  digitalWrite(L298N_IN3_PIN, LOW);
  digitalWrite(L298N_IN4_PIN, LOW);
}

/* Using UltraSonic to get the distance between the car and object */
int getDistance() {
  unsigned long duration;

  digitalWrite(HY_SRF05_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HY_SRF05_TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(HY_SRF05_TRIG_PIN, LOW);

  duration = pulseIn(HY_SRF05_ECHO_PIN, HIGH);

  int distance = int(duration / 29.412 / 2);

  return distance;
}

/* Using Piezo Speaker to alert if having object in range*/
void scanObject() {
  int distance = getDistance();

  if (distance < 20) {
    analogWrite(PIEZO_SPEAKER_PIN, 255);
    delay(100);
  } else if (distance < 50) {
    analogWrite(PIEZO_SPEAKER_PIN, 200);
    delay(500);
  } else if (distance < 100) {
    analogWrite(PIEZO_SPEAKER_PIN, 150);
    delay(700);
  } else {
    analogWrite(PIEZO_SPEAKER_PIN, 0);
  }
}

/* Loop the program */
void loop() {
  // if (Serial.available()) {
  //   direction = Serial.readStringUntil('\n').charAt(0);
  // }

  goStraight(SPEED);

  // switch (direction) {
  //   case 'F':
  //     goStraight(SPEED);
  //     break;
  //   case 'B':
  //     scanObject();
  //     goBack(SPEED);
  //     break;
  //   case 'L':
  //     goRight(SPEED - 50);
  //     goLeft(SPEED);
  //     break;
  //   case 'R':
  //     goLeft(SPEED - 50);
  //     goRight(SPEED);
  //     break;
  //   case 'S':
  //     stop();
  //     break;
  //   default:
  //     SimulatedSerial.println("Error! Can't read the direction.");
  //     delay(5000);
  // }
}
