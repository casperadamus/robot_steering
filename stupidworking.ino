// --- PINS ---
// Motor Driver Pins
#define Lpwm_pin  5  
#define Rpwm_pin  6  
int pinLB = 3;  
int pinLF = 4;  
int pinRB = 7;  
int pinRF = 8;  

// --- 5-SENSOR SETUP ---
const int SENSOR_RIGHT = A0;        
const int SENSOR_LEFT = A1;         
const int SENSOR_MIDDLE_RIGHT = A2; 
const int SENSOR_MIDDLE_LEFT = A3;  
const int SENSOR_MIDDLE = A5;

// --- VARIABLES ---
// Sensor thresholds
const int onBlack = 300;  // Reading for black line from sensor is less than this
const int onWhite = 700;  // Reading for white line is greater than this

// Motor speeds
int base_speed = 150;
int turn_speed = 100;

void setup() {
  // Motor Pins
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
  
  // Sensor Pins (analog pins don't need pinMode in Arduino)
  
  // Serial
  Serial.begin(115200);      
  
  Serial.println("Arduino Ready - PID Line Following Mode");
}

void loop() {
  // Always run line follower
  runLineFollower();
}

// --- HELPER FUNCTIONS ---

bool isOnBlack(int sensor) {
  if (sensor <= onBlack)
    return true;
  else
    return false;
}

bool isOnWhite(int sensor) {
  if (sensor >= onWhite)
    return true;
  else
    return false;
}

// --- LINE FOLLOWING LOGIC ---

void runLineFollower() {
  // Read all 5 sensors
  int sensor1 = analogRead(SENSOR_LEFT);         // Extreme Left
  int sensor2 = analogRead(SENSOR_MIDDLE_LEFT);  // Left
  int sensor3 = analogRead(SENSOR_MIDDLE);       // Middle sensor
  int sensor4 = analogRead(SENSOR_MIDDLE_RIGHT); // Right
  int sensor5 = analogRead(SENSOR_RIGHT);        // Extreme right

  // Extreme left sensor on black - turn left
  if (isOnBlack(sensor1)) {
    turnLeft();
  } 
  else {
    // Middle sensor on black and extreme left on white - go forward
    if (isOnBlack(sensor3) && isOnWhite(sensor1)) {
      moveForward();
    } 
    // Extreme left and middle on white, extreme right on black - turn right
    else if (isOnWhite(sensor1) && isOnWhite(sensor3) && isOnBlack(sensor5)) {
      turnRight();
    } 
    // Left-middle sensor on black - turn left
    else if (isOnBlack(sensor2)) {
      turnLeft();
    } 
    // Right-middle sensor on black - turn right
    else if (isOnBlack(sensor4)) {
      turnRight();
    } 
    // All sensors on white (lost line) - turn in circle to find line
    else if (isOnWhite(sensor1) && isOnWhite(sensor2) && isOnWhite(sensor3) && 
             isOnWhite(sensor4) && isOnWhite(sensor5)) {
      turnCircle();  // Can replace with turnLeft() if gap is present
    }
  }
}

void PID_LINE_FOLLOW() {
  while (1) {
    runLineFollower();
  }
}

// --- MOTOR HELPERS ---

// Motor control function compatible with your motor driver
void motor(int left_speed, int right_speed) {
  // Left motor
  if (left_speed > 0) {
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    analogWrite(Lpwm_pin, constrain(left_speed, 0, 255));
  } else if (left_speed < 0) {
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    analogWrite(Lpwm_pin, constrain(abs(left_speed), 0, 255));
  } else {
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, HIGH);
    analogWrite(Lpwm_pin, 0);
  }

  // Right motor
  if (right_speed > 0) {
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    analogWrite(Rpwm_pin, constrain(right_speed, 0, 255));
  } else if (right_speed < 0) {
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, LOW);
    analogWrite(Rpwm_pin, constrain(abs(right_speed), 0, 255));
  } else {
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, HIGH);
    analogWrite(Rpwm_pin, 0);
  }
}

void rotate_left(int speed) {
  motor(-speed, speed);   
}

void rotate_right(int speed) {
  motor(speed, -speed);  
}

void stopRobot() {
  motor(0, 0);
}

void moveForward() {
  digitalWrite(pinLF, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLB, LOW);
  analogWrite(Lpwm_pin, base_speed);
  analogWrite(Rpwm_pin, base_speed);
}

void turnLeft() {
  digitalWrite(pinLF, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLB, LOW);
  analogWrite(Lpwm_pin, 0);
  analogWrite(Rpwm_pin, turn_speed);
}

void turnRight() {
  digitalWrite(pinLF, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLB, LOW);
  analogWrite(Lpwm_pin, turn_speed);
  analogWrite(Rpwm_pin, 0);
}

void turnCircle() {
  digitalWrite(pinLF, LOW);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinRF, HIGH);
  analogWrite(Lpwm_pin, turn_speed);
  analogWrite(Rpwm_pin, turn_speed);
}
