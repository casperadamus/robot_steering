# Line-Following Robot with Remote Control

Arduino-based line-following robot with dual operating modes: autonomous line tracking using **PD control** with a **5-sensor array** and manual control via ESP32 wireless gamepad.

---

## Features

* **Dual Operating Modes:**
    * **Auto Mode:** PD-controlled line following with 5-sensor array for precise tracking
    * **Manual Mode:** Joystick control via wireless gamepad
  
* **Dynamic Speed Control:**
    * Adjustable base speed via controller buttons (L1/R1)
    * Automatic speed reduction on sharp turns
    * Speed range: 60-255 (PWM)

* **Advanced Line Following:**
    * 5-sensor weighted position calculation
    * PD (Proportional-Derivative) control algorithm
    * Line memory when sensors lose track
    * Adaptive speed for curves

---

## Hardware Requirements

### Microcontrollers
* **Arduino board** (Uno/Nano recommended)
* **ESP32** (for wireless controller communication)

### Components
* **Motor Driver:** Dual H-Bridge motor driver (L298N, TB6612FNG, or similar)
* **Motors:** 2x DC motors with wheels
* **Sensors:** **5x IR line sensors** (digital output)
* **Communication:** SoftwareSerial connection between Arduino and ESP32
* **Power Supply:** Battery packs for motors and Arduino

---

## Wiring & Pinout

### Motor Pins

| Motor | Pin Function | Arduino Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Left Motor** | PWM (Speed) | `5` (`Lpwm_pin`) | PWM capable pin |
| | Direction 1 (Forward) | `4` (`pinLF`) | Digital I/O |
| | Direction 2 (Backward) | `3` (`pinLB`) | Digital I/O |
| **Right Motor** | PWM (Speed) | `6` (`Rpwm_pin`) | PWM capable pin |
| | Direction 1 (Forward) | `8` (`pinRF`) | Digital I/O |
| | Direction 2 (Backward) | `7` (`pinRB`) | Digital I/O |

### Sensor Pins (5-Sensor Array)

| Sensor Position | Arduino Pin | Notes |
| :--- | :--- | :--- |
| **Far Left (FL)** | `A1` (`SENSOR_LEFT`) | Analog pin as Digital Input |
| **Middle Left (ML)** | `A3` (`SENSOR_MIDDLE_LEFT`) | Analog pin as Digital Input |
| **Center (M)** | `A5` (`SENSOR_MIDDLE`) | Analog pin as Digital Input |
| **Middle Right (MR)** | `A2` (`SENSOR_MIDDLE_RIGHT`) | Analog pin as Digital Input |
| **Far Right (FR)** | `A0` (`SENSOR_RIGHT`) | Analog pin as Digital Input |

### Communication Pins

| Function | Arduino Pin | Connection |
| :--- | :--- | :--- |
| **ESP32 RX** | `10` | Connect to ESP32 TX |
| **ESP32 TX** | `11` | Connect to ESP32 RX |

**Note:** Ensure **common ground (GND)** connection between Arduino, Motor Driver, Sensors, and ESP32.

---

## Communication Protocol

The Arduino receives data from ESP32 in a comma-separated format enclosed in angle brackets:

```
<Y,X,Cross,Circle,L1,R1>
```

### Data Packet Format

| Variable | Type | Range | Description |
| :--- | :--- | :--- | :--- |
| **Y** | `int` | -127 to 127 | Joystick Y-axis (Throttle/Forward-Backward) |
| **X** | `int` | -127 to 127 | Joystick X-axis (Steering/Left-Right) |
| **Cross** | `int` | 0 or 1 | Button State (Enable Auto Mode) |
| **Circle** | `int` | 0 or 1 | Button State (Enable Manual Mode) |
| **L1** | `int` | 0 or 1 | Button State (Decrease Speed) |
| **R1** | `int` | 0 or 1 | Button State (Increase Speed) |

**Example:** `<50,-30,0,0,0,1>` means joystick at Y=50, X=-30, R1 pressed

**Serial Configuration:**
* Arduino: 115200 baud (USB Serial Monitor)
* ESP32: 9600 baud (SoftwareSerial)

---

## Control Guide

### Controller Mapping

| Button/Input | Function |
| :--- | :--- |
| **Cross (X)** | Switch to Auto Mode (line following) |
| **Circle (O)** | Switch to Manual Mode (stop + manual control) |
| **L1** | Decrease base speed by 25 |
| **R1** | Increase base speed by 25 |
| **Left Joystick** | Manual driving (Y=forward/back, X=steering) |

### Operating Modes

**Auto Mode (Line Following)**
* Robot autonomously follows a dark line on light surface
* Uses PD control for smooth tracking
* Automatically slows down on sharp curves
* Maintains line even if temporarily lost

**Manual Mode**
* Direct joystick control
* Y-axis: Forward/backward throttle
* X-axis: Left/right steering
* Deadzone of ±15 to prevent drift

---

## Tuning Parameters

### PD Controller
```cpp
float Kp = 25;  // Proportional gain (responsiveness)
float Kd = 15;  // Derivative gain (damping/smoothness)
```

### Speed Settings
```cpp
int BASE_SPEED = 150;      // Default straight-line speed
int MAX_SPEED = 255;       // Maximum motor speed
int MIN_SPEED = 60;        // Minimum to prevent stalling
int SPEED_INCREMENT = 25;  // Speed change per button press
int SPEED_DROP = 50;       // Speed reduction on curves
```

### Tuning Tips
* **Increase Kp:** More aggressive corrections, faster response
* **Decrease Kp:** Gentler corrections, may be slower
* **Increase Kd:** More damping, reduces oscillation
* **Decrease Kd:** Less damping, may oscillate
* **Increase SPEED_DROP:** Slower on curves (more stable)
* **Decrease SPEED_DROP:** Faster on curves (may overshoot)

---

## How It Works

### Line Following Algorithm (PD Control)

**1. Sensor Reading**
* Read 5 digital sensors (0=white, 1=black line)

**2. Position Calculation**
* Calculate weighted position error:
  ```
  Weights: FL=-2000, ML=-1000, M=0, MR=+1000, FR=+2000
  Error = Sum(sensor × weight) / Active sensors
  ```

**3. PD Control**
* Calculate correction based on error:
  ```
  P = error
  D = error - lastError
  correction = (Kp × P) + (Kd × D)
  ```

**4. Speed Adjustment**
* Slow down on sharp turns:
  ```
  if |error| > 1000:
      currentSpeed = BASE_SPEED - SPEED_DROP
  ```

**5. Motor Control**
* Apply correction to motors:
  ```
  leftSpeed = currentSpeed + correction
  rightSpeed = currentSpeed - correction
  ```

### Sensor Logic

The sensors are configured for **Active-HIGH** (digital):
* **1 (HIGH)** = Black Line Detected
* **0 (LOW)** = White Surface Detected

### Manual Control

Tank-style mixing of throttle and steering:
```cpp
leftMotorSpeed = throttle - steering
rightMotorSpeed = throttle + steering
```

---

## Setup Instructions

### 1. Hardware Assembly
* Connect motors to motor driver
* Wire 5 sensors in array with equal spacing
* Connect motor driver to Arduino pins
* Set up SoftwareSerial between Arduino and ESP32
* Ensure common ground connection

### 2. Software Upload
* Install Arduino IDE
* Install SoftwareSerial library (if not included)
* Upload `goodcode.ino` to Arduino
* Configure ESP32 with gamepad receiver code (separate project)

### 3. Calibration
* Place robot on track
* Adjust sensor height (3-5mm from surface recommended)
* Test sensor readings via Serial Monitor (115200 baud)
* Tune Kp/Kd values for your specific track

### 4. Testing
* Open Serial Monitor (115200 baud)
* Verify controller data reception: `Arduino Ready. Waiting for <Y,X,Cross,Circle,L1,R1>...`
* Test Manual Mode first using joystick
* Switch to Auto Mode on line
* Adjust speed using L1/R1 buttons

---

## Troubleshooting

| Issue | Solution |
| :--- | :--- |
| Robot doesn't move | Check motor connections and power supply |
| Oscillates on line | Decrease Kp or increase Kd |
| Loses line on curves | Increase SPEED_DROP or decrease BASE_SPEED |
| Slow response | Increase Kp |
| No controller data | Check ESP32 serial connection (pins 10/11) and baud rate |
| Motors run backwards | Swap forward/backward pins in code |
| Sensors not detecting | Check sensor height and verify digital output |
| Speed won't change | Verify L1/R1 button signals in Serial Monitor |

---

## Code Structure

```
goodcode.ino
├── Pin Definitions & Tuning Parameters
├── Global Variables
├── setup()
│   ├── Pin initialization
│   └── Serial initialization
├── loop()
│   ├── recvWithStartEndMarkers()  // Receive data from ESP32
│   ├── parseData()                // Parse controller input
│   ├── adjustSpeed()              // Handle L1/R1 speed changes
│   └── handleState()              // Switch between modes
├── Control Functions
│   ├── runPDLineFollower()        // Advanced PD line following
│   └── driveRobot()               // Manual joystick control
└── Motor Helpers
    ├── setLeftMotor()
    ├── setRightMotor()
    ├── rotate_left()
    ├── rotate_right()
    └── stopRobot()
```

---

## Usage Instructions

1. **Upload:** Upload `goodcode.ino` to your Arduino board
2. **Power On:** Power on the robot and ESP32/Controller setup
3. **Start-up:** Arduino displays: `Arduino Ready. Waiting for <Y,X,Cross,Circle,L1,R1>...`
4. **Mode Select:**
   * Press **Cross** button → Activate **Auto Mode** (line following)
   * Press **Circle** button → Activate **Manual Mode** (stop + manual control)
5. **Speed Control:**
   * Press **R1** → Increase base speed by 25
   * Press **L1** → Decrease base speed by 25
   * Speed changes are displayed in Serial Monitor
6. **Operation:**
   * **Manual:** Use controller joystick to drive
   * **Auto:** Place robot on black line track - it will follow until mode is switched

---

## License

Open source - feel free to modify and adapt for your projects.

---

## Credits

Developed for Arduino-based line-following robot with ESP32 wireless control integration.
