
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
    *   Connect the flex sensor to the Arduino R4.
    *   Upload an Arduino sketch that reads the sensor value and writes it to the serial port. The Go script expects the Arduino to be on port `/dev/ttyACM0` with a baud rate of 9600.

2.  **Go Environment:**
    *   Install the required Go packages:
        ```bash
        go get github.com/tarm/serial
        go get tinygo.org/x/bluetooth
        ```

3.  **LEGO Spike Prime:**
    *   Ensure the Spike Prime hub is integrated into your SPM structure, powered on, and discoverable via Bluetooth.

## Usage

1.  Run the `main.go` script:
    ```bash
    go run .
    ```

2.  The script will prompt you to confirm that you want to override the program in the specified slot on the Spike Prime. Press `Y` and Enter to continue.

3.  The script will then perform the following steps:
    *   Scan for and connect to the Spike Prime hub.
    *   Perform a calibration sequence to bring the SPM tip close to the sample surface.
    *   Enter a main control loop where it continuously reads sensor data from the Arduino and adjusts the SPM's motors to maintain a constant tip-sample distance.

## Communication Protocol

The communication with the LEGO Spike Prime hub is based on a custom protocol that involves:

*   **COBS (Consistent Overhead Byte Stuffing):** Used to frame messages and avoid special byte values in the data.
*   **CRC32:** A checksum used to ensure the integrity of the data transmitted.
*   **Custom Message Types:** The `messages.go` file defines various message types for different actions, such as file uploads, program execution, and device notifications.

## How it Works

The `main.go` script first establishes a serial connection with the Arduino and a BLE connection with the Spike Prime hub. It then enters a calibration phase where it runs a motor on the SPM to bring the tip into contact with the sample surface, detected by a change in the flex sensor's reading.

After calibration, the script enters a continuous control loop. In this loop, it reads the flex sensor data from the Arduino and adjusts the SPM's motors to maintain a constant setpoint, effectively keeping the tip at a fixed distance from the sample. The script sends Python code to the Spike Prime to control the motors, demonstrating the ability to dynamically program the hub for sophisticated SPM control.
