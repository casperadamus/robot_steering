#include <SoftwareSerial.h>

// --- PINS ---
// Motor Driver Pins
#define Lpwm_pin  5  
#define Rpwm_pin  6  
int pinLB = 3;  
int pinLF = 4;  
int pinRB = 7;  
int pinRF = 8;  

// --- NEW 3-SENSOR SETUP ---
const int SENSOR_LEFT = A0;   // Left Sensor
const int SENSOR_CENTER = A2; // Center Sensor
const int SENSOR_RIGHT = A1;  // Right Sensor

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

// Robot State
bool isFollowing = false;
const int FOLLOW_SPEED = 90;  // Base speed
const int TURN_SPEED = 110;   // Speed when turning

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
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  // Serial
  Serial.begin(115200);      
  ESP32Serial.begin(9600);   
  
  Serial.println("Arduino Ready. Waiting for <Y,X,Cross,Circle>...");
}

void loop() {
  recvWithStartEndMarkers();
  
  if (newData) {
    parseData();
    handleState(); 
    newData = false;
  }
}

// --- LOGIC FUNCTIONS ---

void handleState() {
  if (btnCross == 1) {
    isFollowing = true;
    Serial.println("MODE: Auto (Line Follow)");
  }
  if (btnCircle == 1) {
    isFollowing = false;
    stopRobot(); 
    Serial.println("MODE: Manual (Joystick)");
  }

  if (isFollowing) {
    runLineFollower();
  } else {
    driveRobot(joyY, joyX);
  }
}

void runLineFollower() {
  int L = digitalRead(SENSOR_LEFT);
  int C = digitalRead(SENSOR_CENTER);
  int R = digitalRead(SENSOR_RIGHT);

  // LOGIC: 1 = Black (Line), 0 = White (Floor)
  
  // SITUATION 1: Center sees Line (Perfect)
  // Logic: Go straight
  if (C == 1 && L == 0 && R == 0) {
    setLeftMotor(FOLLOW_SPEED);
    setRightMotor(FOLLOW_SPEED);
  }
  
  // SITUATION 2: Left sees Line
  // Logic: We are drifting Right, so Turn Left
  else if (L == 1) {
    rotate_left(TURN_SPEED);
  }
  
  // SITUATION 3: Right sees Line
  // Logic: We are drifting Left, so Turn Right
  else if (R == 1) {
    rotate_right(TURN_SPEED);
  }
  
  // SITUATION 4: All Black (Intersection) or Center+Left / Center+Right
  // Logic: Prioritize going straight if Center is on, OR stop if it's a finish line.
  // For now, let's keep going straight if Center is involved.
  else if (C == 1) {
    setLeftMotor(FOLLOW_SPEED);
    setRightMotor(FOLLOW_SPEED);
  }
  
  // SITUATION 5: All White (Lost Line)
  // Logic: Stop (or you can code it to spin in circle to find line)
  else if (L == 0 && C == 0 && R == 0) {
    stopRobot();
  }
}

void driveRobot(int y, int x) {
  // Deadzone
  if (abs(y) < 15) y = 0;
  if (abs(x) < 15) x = 0;

  int throttle = y * 2; 
  int steering = x * 2; 

  // Fixed steering direction from previous chat
  int leftMotorSpeed = throttle - steering;  
  int rightMotorSpeed = throttle + steering; 

  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  setLeftMotor(leftMotorSpeed);
  setRightMotor(rightMotorSpeed);
}

// --- PARSING ---

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
    strtokIndx = strtok(receivedChars,","); joyY = atoi(strtokIndx);     
    strtokIndx = strtok(NULL, ","); joyX = atoi(strtokIndx);
    strtokIndx = strtok(NULL, ","); btnCross = atoi(strtokIndx);
    strtokIndx = strtok(NULL, ","); btnCircle = atoi(strtokIndx);
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
