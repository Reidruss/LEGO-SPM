
# LEGO Scanning Probe Microscope (SPM) Controller

This project provides a set of Go scripts to control a LEGO-based Scanning Probe Microscope (SPM). The scripts facilitate communication between an Arduino R4, which acts as the sensor controller, and a LEGO Spike Prime hub, which controls the microscope's mechanics.

## Project Overview

The primary goal of this project is to create a robust communication and control system for a LEGO SPM. The system allows the SPM's tip to be precisely controlled based on feedback from a sensor connected to an Arduino. In the current implementation, a flex sensor is used to provide feedback for maintaining a constant distance between the SPM tip and the sample surface.

The communication and control flow is as follows:

1.  A flex sensor, acting as the feedback mechanism for the SPM, is connected to the Arduino R4.
2.  The Arduino reads the sensor values and sends them to the Go control script via serial communication.
3.  The Go script establishes a Bluetooth LE (BLE) connection with the LEGO Spike Prime hub, which is the core of the SPM's mechanical structure.
4.  The Go script sends commands to the Spike Prime to control the motors, adjusting the SPM's tip position based on the sensor values received from the Arduino.

## Hardware and Software Requirements

### Hardware

*   A LEGO-based Scanning Probe Microscope structure
*   LEGO Spike Prime Hub
*   Arduino R4
*   Flex sensor
*   A computer with Go installed and a Bluetooth adapter

### Software

*   [Go](https://golang.org/doc/install)
*   [Arduino IDE](https://www.arduino.cc/en/software)
*   A custom Arduino sketch to read the flex sensor and send data over serial.

## Go Scripts

The LEGO SPM controller is composed of the following Go scripts:

*   `main.go`: The main script that orchestrates the communication between the Arduino and the Spike Prime. It handles serial communication, BLE connection, and the main control loop for the SPM.
*   `messages.go`: Defines the data structures and serialization/deserialization functions for the communication protocol with the Spike Prime.
*   `cobs.go`: Implements Consistent Overhead Byte Stuffing (COBS) for framing messages sent to the Spike Prime.
*   `crc.go`: Implements CRC32 for data integrity checks.

## Setup

1.  **Arduino:**
    *   Connect the flex sensor to the Arduino R4. A screenshot is given below of the wiring setup for the flex sensor.
    *   Upload an Arduino sketch that reads the sensor value and writes it to the serial port. The Go script will scan ports on macOs, Linux, and Windows.
<img width="864" height="540" alt="Screenshot from 2025-12-03 16-19-41" src="https://github.com/user-attachments/assets/f5307260-8c9a-4884-b9b7-5408a4052ddc" />

2.  **Go Environment:**
    *   Install the required Go packages:
        ```bash
        go get github.com/tarm/serial
        go get tinygo.org/x/bluetooth
        ```
    *   These will install automatically with:
        ```bash
        go build .
        ```

3.  **LEGO Spike Prime:**
    *   Ensure the Spike Prime hub is integrated into your SPM structure, powered on, and connected to your machine, via Bluetooth.

## Configuration
Before running the script, you must calibrate the constants in `main.go` to match your specific hardware setup.

1. **Motor Speed (`MOTOR_SPEED`)
   * **Recommendation:** Set between **40 and 60**.
   * **< 40:** Risk of motor "cogging" (stuttering) and stiction, preventing fine adjustments.
   * **> 60:** Risk of overshoot; the SPM may struggle to settle within the setpoint range.
2. **Thresholds (`SETPOINT_RANGE`)** to determine your values:
   1. Read the baseline resistance value of the sensor when the tip is not in contact with the sample.
   2. Set `SETPOINT_RANGE_LOW` to a value approximately **1000–2000 units greater** than your baseline.
   3. Set `SETPOINT_RANGE_HIGH` to define the upper limit of the acceptable range.

Open `main.go` and modify the constants block:
```go
const (
    SETPOINT_RANGE_LOW  = 3750 // Baseline + Buffer
    SETPOINT_RANGE_HIGH = 6000 // Upper limit
    MOTOR_SPEED         = 50   // Recommended: 40 to 60
)
```


## Usage

1.  Run the `main.go` script:
    ```bash
    go run .
    ```

2.  The script will prompt you to confirm that you want to override the program in the specified slot on the Spike Prime. Press `Y` and Enter to continue.

## Runtime Process
Once confirmed, the script performs the following sequence:
1. **Connection:** Scans for the Arduino Serial Port and the LEGO Spike Prime hub.
2. **Calibration:** Performs a sequence to bring the SPM tip into contact with the sample surface.
3. **Control Loop:** Enters the main feedback loop. It continuously reads sensor data from the Arduino and adjusts the SPM's motors to maintain the tip-sample distance within your configured thresholds.
  * **If Value < Low:** The tip is pressing too hard (or too close). The script runs Motors A & C to retract/tighten the assembly.
  * **If Value > High:** The tip is too far (or too loose). The script runs Motors A & C to extend/loosen the assembly.
  * **If Value is within Range:** The tip is perfectly positioned. The script runs Motor B to scan across the surface (X/Y axis).

## Communication Protocol

The communication with the LEGO Spike Prime hub is based on a custom protocol that involves:

*   **COBS (Consistent Overhead Byte Stuffing):** Used to frame messages and avoid special byte values in the data.
*   **CRC32:** A checksum used to ensure the integrity of the data transmitted.
*   **Custom Message Types:** The `messages.go` file defines various message types for different actions, such as file uploads, program execution, and device notifications.

## How it Works

The `main.go` script first establishes a serial connection with the Arduino and a BLE connection with the Spike Prime hub. It then enters a calibration phase where it runs a motor on the SPM to bring the tip into contact with the sample surface, detected by a change in the flex sensor's reading.

After calibration, the script enters a continuous control loop. In this loop, it reads the flex sensor data from the Arduino and adjusts the SPM's motors to maintain a constant setpoint, effectively keeping the tip at a fixed distance from the sample. The script sends microPython code to the Spike Prime to control the motors, demonstrating the ability to dynamically program the hub for sophisticated SPM control.
