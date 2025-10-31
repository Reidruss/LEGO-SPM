'''
This script was developed by Reid Russell

'''

import sys
import serial
from typing import cast, TypeVar
import queue
import threading
import time

TMessage = TypeVar("TMessage", bound="BaseMessage")

import cobs
from messages import *
from crc import crc

import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData


SCAN_TIMEOUT = 25.0

ARDUINO_PORT = 'COM3'

ARDUINO_BAUDRATE = 9600

# PID Controller parameters
KP = 0.5
KI = 0.1
KD = 0.05
SETPOINT_RANGE = (1200, 1400)  # Target flex sensor value range

'''
The SPIKE™ Prime BLE UUIDs
 - service
 - UUID that will recieve data
 - UUID that will transmit data
'''
SERVICE = "0000fd02-0000-1000-8000-00805f9b34fb"
RX_CHAR = "0000fd02-0001-1000-8000-00805f9b34fb"
TX_CHAR = "0000fd02-0002-1000-8000-00805f9b34fb"


DEVICE_NOTIFICATION_INTERVAL_MS = 5000

PROGRAM_SLOT = 2

PROGRAM_SAMPLE = \
"""import motor
from hub import port
import time

motor.run_for_degrees(port.E, 360, 100)
time.sleep(0.5)
""".encode("utf8")

class PIDController:
    """A simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float, setpoint_range: tuple[float, float]):
        self.kp = kp # Proportional gain
        self.ki = ki # Integral gain
        self.kd = kd # Derivative gain
        self.setpoint_range = setpoint_range
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value: float) -> float:
        """Calculate the PID output."""
        lower_bound, upper_bound = self.setpoint_range

        if lower_bound <= current_value <= upper_bound:
            error = 0
            self.integral = 0
        elif current_value < lower_bound:
            error = lower_bound - current_value
            self.integral += error
        else:
            error = upper_bound - current_value
            self.integral += error

        derivative = error - self.previous_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output


PROGRAM_RAISE_AND_RESET = \
"""import motor
from hub import port
import time

motor.run_for_degrees(port.E, -720, 100)
time.sleep(5)
""".encode("utf8")


answer = input(
    f"This example will override the program in slot {PROGRAM_SLOT} of the first hub found. Do you want to continue? [Y/n] "
)

if answer.strip().lower().startswith("n"):
    print("Aborted by user.")
    sys.exit(0)

stop_event = asyncio.Event()
# Add a global queue for serial data
serial_queue = queue.Queue()

def serial_reader(ser, stop_event):
    """Read data from serial port and put it into a queue."""
    while not stop_event.is_set():
        try:
            ser.write(b'g')
            line = ser.readline().decode("ascii").strip()
            if line:
                serial_queue.put(line)
            # Add a small delay to prevent busy-waiting
            time.sleep(0.01)
        except serial.SerialException:
            print("Arduino disconnected. Retrying...")
            time.sleep(2)
            continue

async def main():
    def match_service_uuid(device: BLEDevice, adv: AdvertisementData) -> bool:
        return SERVICE.lower() in adv.service_uuids

    print(f"\nScanning for {SCAN_TIMEOUT} seconds, please wait...")
    device = await BleakScanner.find_device_by_filter(
        filterfunc=match_service_uuid, timeout=SCAN_TIMEOUT
    )

    if device is None:
        print("No hubs detected. Ensure that a hub is within range, turned on, and awaiting connection.")
        sys.exit(1)

    device = cast(BLEDevice, device)
    print(f"Hub detected! {device}")

    def on_disconnect(client: BleakClient) -> None:
        print("Connection lost.")
        stop_event.set()

    print("Connecting...")
    ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)

    # Start the serial reader thread
    reader_thread = threading.Thread(target=serial_reader, args=(ser, stop_event))
    reader_thread.daemon = True
    reader_thread.start()

    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        print("Connected!\n")

        service = client.services.get_service(SERVICE)
        rx_char = service.get_characteristic(RX_CHAR)
        tx_char = service.get_characteristic(TX_CHAR)

        pending_response: tuple[int, asyncio.Future] = (-1, asyncio.Future())

        def on_data(_: BleakGATTCharacteristic, data: bytearray) -> None:
            if data[-1] != 0x02:
                un_xor = bytes(map(lambda x: x ^ 3, data))
                print(f"Received incomplete message:\n {un_xor}")
                return
            data = cobs.unpack(data)
            try:
                message = deserialize(data)
                print(f"Received: {message}")

                if message.ID == pending_response[0]:
                    pending_response[1].set_result(message)

                if isinstance(message, DeviceNotification):
                    updates = list(message.messages)
                    updates.sort(key=lambda x: x[1])
                    lines = [f" - {x[0]:<10}: {x[1]}" for x in updates]
                    print("\n".join(lines))

            except ValueError as e:
                print(f"Error: {e}")

        await client.start_notify(tx_char, on_data)

        async def send_request_with_tracking(message, response_type):
            """Send message and await response."""
            nonlocal pending_response
            pending_response = (response_type.ID, asyncio.Future())
            payload = message.serialize()
            frame = cobs.pack(payload)
            await client.write_gatt_char(rx_char, frame, response=False)
            return await pending_response[1]

        # === INITIALIZE HUB CONNECTION ===
        info_response: InfoResponse = await send_request_with_tracking(InfoRequest(), InfoResponse)
        notif_resp = await send_request_with_tracking(
            DeviceNotificationRequest(DEVICE_NOTIFICATION_INTERVAL_MS),
            DeviceNotificationResponse
        )
        if not notif_resp.success:
            print("Error: failed to enable notifications")
            sys.exit(1)

        # === UPLOAD + RUN HELPER ===
        async def upload_and_run(program_code: bytes):
            """Upload a program to the hub and start it."""
            program_crc = crc(program_code)

            start_upload = await send_request_with_tracking(
                StartFileUploadRequest("program.py", PROGRAM_SLOT, program_crc),
                StartFileUploadResponse
            )

            if not start_upload.success:
                print("Start upload failed")
                return False

            running_crc = 0
            for i in range(0, len(program_code), info_response.max_chunk_size):
                chunk = program_code[i:i + info_response.max_chunk_size]
                running_crc = crc(chunk, running_crc)

                chunk_resp = await send_request_with_tracking(
                    TransferChunkRequest(running_crc, chunk),
                    TransferChunkResponse
                )
                
                if not chunk_resp.success:
                    print("Chunk failed")
                    return False

            start_resp = await send_request_with_tracking(
                ProgramFlowRequest(stop=False, slot=PROGRAM_SLOT),
                ProgramFlowResponse
            )
            if not start_resp.success:
                print("Error: failed to start program")
                return False

            print("Program started successfully.")
            return True

        # === MAIN LOOP ===
        async def monitor_flex_sensor():
            """Continuously read Arduino data and adjust probe height using PID controller."""
            pid = PIDController(KP, KI, KD, SETPOINT_RANGE)
            motor_adjusting = False

            while not stop_event.is_set():
                if not motor_adjusting:
                    try:
                        # Get data from the queue
                        line = serial_queue.get_nowait()
                    except queue.Empty:
                        await asyncio.sleep(0.05)
                        continue

                    try:
                        current_value = float(line)
                    except ValueError:
                        continue

                    print(f"Flex sensor: {current_value}")

                    # Calculate PID output
                    output = pid.update(current_value)

                    # Clamp the output to a reasonable range
                    output = max(min(output, 300), -300)

                    # If the output is significant, move the motor
                    if abs(output) > 10:
                        motor_adjusting = True
                        degrees = int(output) % 20
                        program_code = f"""import motor
from hub import port
import time

motor.run_for_degrees(port.A, {degrees}, 400)
time.sleep(0.1)
""".encode("utf8")
                        print(f"Adjusting motor by {degrees} degrees.")
                        await upload_and_run(program_code)

                        # Clear the queue after motor adjustment
                        with serial_queue.mutex:
                            serial_queue.queue.clear()
                        motor_adjusting = False

                await asyncio.sleep(1)

        await monitor_flex_sensor()
        await stop_event.wait()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user.")
        stop_event.set()
