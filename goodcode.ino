#include <SoftwareSerial.h>

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
int MAX_SPEED = 255;
int MIN_SPEED = 60;       // Minimum speed to prevent stalling
int SPEED_INCREMENT = 25; // approx 10% change per press

// *** NEW: CURVE HANDLING ***
int SPEED_DROP = 50;      // How much to slow down on sharp turns

int lastError = 0;

// --- 4-SENSOR SETUP ---
const int SENSOR_RIGHT = A0;        
const int SENSOR_LEFT = A1;        
const int SENSOR_MIDDLE_RIGHT = A2;
const int SENSOR_MIDDLE_LEFT = A3;  
const int SENSOR_MIDDLE = A5;

// Communication with ESP32
SoftwareSerial ESP32Serial(10, 11);

// --- VARIABLES ---
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

// Data from Controller
int joyY = 0;
int joyX = 0;
int btnCross = 0;  
int btnCircle = 0;
int btnL1 = 0;
int btnR1 = 0;

// Button State Tracking
int lastL1State = 0;
int lastR1State = 0;

// Robot State
bool isFollowing = true;

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
  pinMode(SENSOR_MIDDLE, INPUT); // Added explicit input mode for middle

  // Serial
  Serial.begin(115200);      
  ESP32Serial.begin(9600);  
 
  Serial.println("Arduino Ready. Waiting for <Y,X,Cross,Circle,L1,R1>...");
}

void loop() {
  recvWithStartEndMarkers();
 
  if (newData) {
    parseData();
    adjustSpeed(); // Check for speed changes via Controller
    handleState();
    newData = false;
  }
}

// --- SPEED LOGIC (MANUAL ADJUSTMENT) ---
void adjustSpeed() {
  // Check R1 (Increase Speed)
  if (btnR1 == 1 && lastR1State == 0) {
    BASE_SPEED += SPEED_INCREMENT;
    if (BASE_SPEED > MAX_SPEED) BASE_SPEED = MAX_SPEED;
    Serial.print("Base Speed INCREASED to: ");
    Serial.println(BASE_SPEED);
  }

  // Check L1 (Decrease Speed)
  if (btnL1 == 1 && lastL1State == 0) {
    BASE_SPEED -= SPEED_INCREMENT;
    if (BASE_SPEED < MIN_SPEED) BASE_SPEED = MIN_SPEED;
    Serial.print("Base Speed DECREASED to: ");
    Serial.println(BASE_SPEED);
  }

  lastR1State = btnR1;
  lastL1State = btnL1;
}

// --- LOGIC FUNCTIONS ---

void handleState() {
  if (btnCross == 1) {
    isFollowing = true;
    // Serial.println("MODE: Auto"); // Comment out if it slows down loop
  }
  if (btnCircle == 1) {
    isFollowing = false;
    stopRobot();
    Serial.println("MODE: Manual");
  }

  if (isFollowing) {
    runPDLineFollower();
  } else {
    driveRobot(joyY, joyX);
  }
}

// *** THIS IS THE UPDATED FUNCTION ***
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

void driveRobot(int y, int x) {
  // Deadzone
  if (abs(y) < 15) y = 0;
  if (abs(x) < 15) x = 0;

  float speedFactor = (float)BASE_SPEED / 255.0;
 
  int throttle = (y * 2) * speedFactor;
  int steering = (x * 2) * speedFactor;

  int leftMotorSpeed = throttle - steering;  
  int rightMotorSpeed = throttle + steering;

  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  setLeftMotor(leftMotorSpeed);
  setRightMotor(rightMotorSpeed);
}

// --- PARSING (Unchanged) ---

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (ESP32Serial.available() > 0 && newData == false) {
        rc = ESP32Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) ndx = numChars - 1;
            } else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char * strtokIndx;
   
    strtokIndx = strtok(receivedChars,",");
    if(strtokIndx != NULL) joyY = atoi(strtokIndx);    
   
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) joyX = atoi(strtokIndx);
   
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) btnCross = atoi(strtokIndx);
   
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) btnCircle = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) btnL1 = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) btnR1 = atoi(strtokIndx);
}

// --- MOTOR HELPERS (Unchanged) ---

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

void rotate_left(int speed) {
  setLeftMotor(-speed);  
  setRightMotor(speed);  
}

void rotate_right(int speed) {
  setLeftMotor(speed);    
  setRightMotor(-speed);  
}

void stopRobot() {
  setLeftMotor(0);
  setRightMotor(0);
}
