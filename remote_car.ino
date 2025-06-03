// #include <>

// Using L298N to control the electricity
const int L298N_IN1_PIN = 2;
const int L298N_IN2_PIN = 3;

// Using UltraSonic to calculate the distance between the car to object
const int HY_SRF05_TRIG_PIN = 5;
const int HY_SRF05_ECHO_PIN = 6;

// Using Relay Module to control wheels
const int RELAY_MODULE_PIN = 7;

/*
  Direction of the car

  Note:
    W: Go Straight
    S: Go Back
    A: Go Left
    D: Go Right

  Default: 'W'
*/
char direction = 'W';

/* Initializes the setting up */
void setup() {

  // Initialize Serial Monitor's port
  Serial.begin(9600);

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

  // Initialize UltraSonic PIN
  pinMode(HY_SRF05_TRIG_PIN, OUTPUT);
  pinMode(HY_SRF05_ECHO_PIN, INPUT);

  // Initialize Relay Module PIN
  pinMode(RELAY_MODULE_PIN, OUTPUT);

}

void goStraight() {
  // The electricity will be from L298N_IN1_PIN to L298N_IN2_PIN
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);

  digitalWrite(RELAY_MODULE_PIN, HIGH);
}

void goBack() {
  // The electricity will be from L298N_IN1_PIN to L298N_IN2_PIN
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);

  digitalWrite(RELAY_MODULE_PIN, HIGH);
}

void goLeft() {
  // The electricity will be from L298N_IN2_PIN to L298N_IN1_PIN
  digitalWrite(L298N_IN1_PIN, LOW);
  digitalWrite(L298N_IN2_PIN, HIGH);

  digitalWrite(RELAY_MODULE_PIN, LOW);
}

void goRight() {
  // The electricity will be from L298N_IN1_PIN to L298N_IN2_PIN
  digitalWrite(L298N_IN1_PIN, HIGH);
  digitalWrite(L298N_IN2_PIN, LOW);

  digitalWrite(RELAY_MODULE_PIN, LOW);
}

/* Using UltraSonic to get the distance between the car and entity */
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

/* Loop the program */
void loop() {
  switch direction:
    case 'W':
      goStraight();
      break;
    case 'S':
      goBack();
      break;
    case 'A':
      goLeft();
      break;
    case 'D':
      goRight();
      break;
    default:
      Serial.println("Error! Can't read the direction.");
      delay(5000);
}
