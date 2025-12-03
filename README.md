# Dual-Mode Line Following Robot (Arduino Mega / Uno)

This project controls a mobile robot using an **Arduino** board (Uno or Mega) that communicates with an **ESP32** (or similar Bluetooth/Wi-Fi module) to receive commands from a **PS4-style controller**. The robot features a dual-mode operation: **Manual Drive** via the joystick and **Autonomous Line Following** using three infrared (IR) sensors.



---

## Features

* **Dual-Mode Control:**
    * **Manual Mode:** Direct control of movement using an external controller's joystick (via the ESP32 bridge).
    * **Autonomous Mode:** Robot follows a black line on a white surface using a 3-sensor array.
* **Differential Drive:** Uses two DC gear motors for movement and turning.
* **Serial Communication:** Employs `SoftwareSerial` to receive structured data (joystick values and button presses) from the ESP32.
* **Simple State Management:** Uses dedicated buttons (`Cross` and `Circle`) to switch between Manual and Auto modes.

---

## Hardware Requirements

* **Microcontroller:** Arduino Uno or Arduino Mega.
* **Motor Driver:** L298N, TB6612FNG, or similar H-Bridge driver.
* **Motors:** Two DC Gear Motors (e.g., geared TT motors).
* **Line Sensors:** Three **Infrared (IR) Tracking Sensors** (Digital Output).
* **Communication Module:** ESP32, ESP8266, or HC-05 (needs separate code to read the PS4 controller and send the data packet).
* **Power Supply:** Battery packs for the motors and the Arduino.

---

## Wiring & Pinout

The following table shows the pin assignments used in the Arduino sketch.

| Component | Pin Function | Arduino Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Left Motor** | PWM (Speed) | `5` (`Lpwm_pin`) | PWM capable pin |
| | Direction 1 (Forward) | `4` (`pinLF`) | Digital I/O |
| | Direction 2 (Backward) | `3` (`pinLB`) | Digital I/O |
| **Right Motor** | PWM (Speed) | `6` (`Rpwm_pin`) | PWM capable pin |
| | Direction 1 (Forward) | `8` (`pinRF`) | Digital I/O |
| | Direction 2 (Backward) | `7` (`pinRB`) | Digital I/O |
| **Sensor Left** | Digital Input | `A0` (`SENSOR_LEFT`) | Analog pin used as Digital Input |
| **Sensor Center** | Digital Input | `A2` (`SENSOR_CENTER`) | Analog pin used as Digital Input |
| **Sensor Right** | Digital Input | `A1` (`SENSOR_RIGHT`) | Analog pin used as Digital Input |
| **ESP32/RX** | SoftwareSerial RX (Arduino TX) | `11` | Connect to ESP32 TX |
| **ESP32/TX** | SoftwareSerial TX (Arduino RX) | `10` | Connect to ESP32 RX |

**Note:** Ensure the **GND** of the Arduino, Motor Driver, Sensors, and the ESP32 are all **connected together (common ground)**.

---

## Software & Configuration

### 1. The Data Packet Format

The Arduino expects data from the ESP32 (or external controller) in a specific, comma-separated format, enclosed in angle brackets:

$$\langle Y, X, Cross, Circle \rangle$$

| Variable | Type | Range | Description |
| :--- | :--- | :--- | :--- |
| **Y** | `int` | $-255$ to $255$ | Joystick Y-axis (Throttle/Forward-Backward) |
| **X** | `int` | $-255$ to $255$ | Joystick X-axis (Steering/Left-Right) |
| **Cross** | `int` | $0$ or $1$ | Button State (Activates Line Following Mode) |
| **Circle** | `int` | $0$ or $1$ | Button State (Activates Manual Drive Mode) |

**You must ensure your ESP32 code sends data in this exact format and baud rate.**

### 2. Sensor Logic

The `runLineFollower` function is configured for **Active-LOW** sensors, meaning:

* **`1` (HIGH) = Black Line Detected**
* **`0` (LOW) = White Floor Detected**

If your sensors output the opposite, you need to change the logic checks (e.g., `if (C == 0 && L == 1 && R == 1)`).

### 3. Line Following Behavior

The robot uses a simple **proportional-like** algorithm:

| Situation | L | C | R | Action |
| :--- | :--- | :--- | :--- | :--- |
| **Straight** | 0 | 1 | 0 | **Go Straight** (`FOLLOW_SPEED`) |
| **Drifting Right** | 1 | 0 | 0 | **Turn Left** (`rotate_left(TURN_SPEED)`) |
| **Drifting Left** | 0 | 0 | 1 | **Turn Right** (`rotate_right(TURN_SPEED)`) |
| **Lost Line** | 0 | 0 | 0 | **Stop** (`stopRobot()`) |
| **Intersection** | 1 | 1 | 1 | **Go Straight** (Prioritizing Center) |

---

## Usage Instructions

1.  **Upload:** Upload the sketch to your Arduino board.
2.  **Power On:** Power on the robot and the ESP32/Controller setup.
3.  **Start-up:** The Arduino waits for data: `Arduino Ready. Waiting for <Y,X,Cross,Circle>...`
4.  **Mode Select:**
    * Press the **Cross** button on the controller to activate **Autonomous Line Following** mode.
    * Press the **Circle** button on the controller to activate **Manual Drive** mode.
5.  **Operation:**
    * **Manual:** Use the controller's analog joystick to drive.
    * **Autonomous:** Place the robot on a black line track. It will begin following the line until the mode is switched.
