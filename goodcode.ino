// --- PINS ---
// Motor Driver Pins
#define Lpwm_pin  5  
#define Rpwm_pin  6  
int pinLB = 3;  
int pinLF = 4;  
int pinRB = 7;  
int pinRF = 8;  

// --- TUNING PARAMETERS ---
float Kp = 25;  
float Kd = 15;  

// --- SPEED CONTROL VARIABLES ---
int BASE_SPEED = 150;     // Starting speed (Straight line speed)
int MAX_SPEED = 150;
int MIN_SPEED = 100;       // Minimum speed to prevent stalling

// *** CURVE HANDLING ***
int SPEED_DROP = 50;      // How much to slow down on sharp turns

int lastError = 0;

// --- 5-SENSOR SETUP ---
const int SENSOR_RIGHT = A0;        
const int SENSOR_LEFT = A1;        
const int SENSOR_MIDDLE_RIGHT = A2;
const int SENSOR_MIDDLE_LEFT = A3;  
const int SENSOR_MIDDLE = A5;

void setup() {
  // Motor Pins
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
 
  // Sensor Pins
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_MIDDLE_LEFT, INPUT);
  pinMode(SENSOR_MIDDLE_RIGHT, INPUT);
  pinMode(SENSOR_MIDDLE, INPUT);

  // Serial
  Serial.begin(115200);      
 
  Serial.println("Arduino Ready - Line Following Mode");
}

void loop() {
  runPDLineFollower();
}

// *** LINE FOLLOWING FUNCTION ***
void runPDLineFollower() {
  // 1. READ SENSORS
  int FL = digitalRead(SENSOR_LEFT);        
  int ML = digitalRead(SENSOR_MIDDLE_LEFT);
  int M = digitalRead(SENSOR_MIDDLE);      
  int MR = digitalRead(SENSOR_MIDDLE_RIGHT);
  int FM = digitalRead(SENSOR_RIGHT);      

  // 2. CALCULATE POSITION
  int error = 0;
  int activeSensors = FL + ML + M + MR + FM;
 
  if (activeSensors > 0) {
    // Weights: -2000, -1000, 0, 1000, 2000
    long weightedSum = (FL * -2000) + (ML * -1000) + (M * 0) + (MR * 1000) + (FM * 2000);
    error = weightedSum / activeSensors;
  } else {
    // Memory: If lost, steer hard in direction of last known error
    if (lastError > 0) error = 2500;
    else error = -2500;
  }

  // 3. CALCULATE PID VALUE
  int P = error;
  int D = error - lastError;
  int correction = (Kp * P) + (Kd * D);
 
  lastError = error;

  // 4. DYNAMIC SPEED THROTTLING (The New Logic)
  // We use a temporary variable for speed so we don't mess up the global BASE_SPEED
  int currentSpeed = BASE_SPEED;

  // If error is greater than 1000 (meaning we are not centered), slow down.
  // 1000 usually means the middle sensor is off the line and a side sensor is on.
  if (abs(error) > 1000) {
      currentSpeed = BASE_SPEED - SPEED_DROP;
     
      // Ensure we don't drop below minimum or the robot will stall in the turn
      if (currentSpeed < MIN_SPEED) {
        currentSpeed = MIN_SPEED;
      }
  }

  // 5. APPLY TO MOTORS
  int leftMotorSpeed = currentSpeed + correction;
  int rightMotorSpeed = currentSpeed - correction;

  // 6. CONSTRAIN SPEEDS
  leftMotorSpeed = constrain(leftMotorSpeed, -MAX_SPEED, MAX_SPEED);
  rightMotorSpeed = constrain(rightMotorSpeed, -MAX_SPEED, MAX_SPEED);

  // 7. DRIVE
  setLeftMotor(leftMotorSpeed);
  setRightMotor(rightMotorSpeed);
}

// --- MOTOR HELPERS ---

void setLeftMotor(int speed) {
  if (speed > 0) {
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    analogWrite(Lpwm_pin, speed);
  } else if (speed < 0) {
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    analogWrite(Lpwm_pin, abs(speed));
  } else {
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, HIGH);
    analogWrite(Lpwm_pin, 0);
  }
}

void setRightMotor(int speed) {
  if (speed > 0) {
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    analogWrite(Rpwm_pin, speed);
  } else if (speed < 0) {
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, LOW);
    analogWrite(Rpwm_pin, abs(speed));
  } else {
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, HIGH);
    analogWrite(Rpwm_pin, 0);
  }
}

void stopRobot() {
  setLeftMotor(0);
  setRightMotor(0);
}
