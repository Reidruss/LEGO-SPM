import sys
from typing import cast, TypeVar

TMessage = TypeVar('TMessage', bound='BaseMessage')

import cobs
from messages import *
from crc import crc

import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

SEARCH_TIMEOUT = 25.0
DEVICE_NOTIFICATION_INTERVAL_MS = 5000
DEVICE_PROGRAM_SLOT = 0

'''
Add logging to this application
'''


PROGRAM_TO_SEND = \
"""#
import motor
from hub import port
import time

motor.run_for_degrees(port.B, 360, 50)
time.sleep(0.5)
""".encode('utf8')


'''
The LEGO® SPIKE™ Prime Hub exposes a BLE GATT service that contains two characteristics:
- RX (for receiving data)
- TX (for transmitting data)

The dict below cotains the UUIDS for the service and characteristics.
'''
LEGO_SPIKE_UUIDs = {
    'Service' : '0000FD02-0000-1000-8000-00805F9B34FB',
    'RX'      : '0000FD02-0001-1000-8000-00805F9B34FB',
    'TX'      : '0000FD02-0002-1000-8000-00805F9B34FB'
}

user_conscent = input(
    f"This example will override the program in slot {DEVICE_PROGRAM_SLOT} of the first hub found. Do you want to continue? [Y/n] "
)
if user_conscent.strip().lower().startswith("n"):
    print("Aborted by user.")
    sys.exit(0)

stop_event = asyncio.Event()

def _match_service_uuid(device: BLEDevice, adv: AdvertisementData) -> bool:
    return LEGO_SPIKE_UUIDs['Service'].lower() in [u.lower() for u in adv.service_uuids]

async def _scan_for_device() -> BLEDevice:
    print(f"\nScanning for {SEARCH_TIMEOUT} seconds, please wait...")
    return await BleakScanner.find_device_by_filter(filterfunc=_match_service_uuid, timeout=SEARCH_TIMEOUT)

def _on_disconnect(client: BleakClient) -> None:
    print(f"Disconnected from {client.address}")
    stop_event.set()


async def main():
    try:
        device = await _scan_for_device()
    except Exception as e:
        print(f"Error: establishing control of bluetooth {e}")
        print("Ensure bluetooth is enabled in settings before runnning the program.")
        sys.exit(1)
    
    if device is None:
        print("No devices detected. Ensure that a device is within range, turned on, and awaiting connection.")
        sys.exit(1)

    device = cast(BLEDevice, device)
    
    print("Device Detected!")
    print("Connecting...")
    async with BleakClient(device, disconnected_callback=_on_disconnect) as client:
        print(f"Connected to {client.address}")

        await client.get_services()
        service = client.services.get_service(LEGO_SPIKE_UUIDs['Service'])
        rx = service.get_characteristic(LEGO_SPIKE_UUIDs['RX'])
        tx = service.get_characteristic(LEGO_SPIKE_UUIDs['TX'])

        pending_responses: dict[int, asyncio.Future] = {}

        def on_data(_: BleakGATTCharacteristic, data: bytearray) -> None:
            if data[-1] != 0x02:
                '''
                TODO:
                    - Implement buffering here so that the program can handle fragmentation correctly.
                    - My thought: Add a bytearray buffer to accumulate unitl a frame delimiter is detected.
                '''
                un_xor = bytes(map(lambda x: x ^ 3, data))
                print(f"Recieved incomplete message:\n {un_xor}")
                return

            data = cobs.unpack(data)

            try:
                message = deserialize(data)
                print(f"Received: {message}")

                if message.ID in pending_responses:
                    pending_responses[message.ID].set_result(message)
                    del pending_responses[message.ID]

                if isinstance(message, DeviceNotification):
                    # sort the messages and print
                    updates = list(message.messages)
                    updates.sort(key=lambda x:x[1])
                    lines: list[str] = [f" - {x[0]:<10}" for x in updates]
                    print("\n".join(lines))
            except Exception as e:
               print(f"Error while handling message: {e}")


        await client.start_notify(tx, on_data)

        info_response: InfoResponse = None

        '''
        Serialize and pack a message, then send it to the device.
        '''
        async def send_message(message: BaseMessage) -> None:
            print(f"Sending {message}")

            payload = message.serialize()
            frame = cobs.pack(payload)

            packet_size = info_response.max_packet_size if info_response else len(frame)

            for i in range(0, len(frame), packet_size):
                packet = frame[i : i + packet_size]
                await client.write_gatt_char(rx, packet, response=False)

        '''
        Sending a message and waiting for an appropriate response.
        '''
        async def send_request(message: BaseMessage, response_type: type[TMessage]) -> TMessage:
            fut = asyncio.Future()
            pending_responses[response_type.ID] = fut
            await send_message(message)
            return await fut
        

        '''
        The first message should be an info request.
        The response will contain important information about the device
        and how to communicate with it.
        '''
        info_request = await send_request(InfoRequest(), InfoResponse)

        notification_response = await send_request(
            DeviceNotificationRequest(DEVICE_NOTIFICATION_INTERVAL_MS),
            DeviceNotificationRequest
        )
        if not notification_response.success:
            print("Error: failed to enable notifications")
            sys.exit(1)

        clear_response = await send_request(ClearSlotRequest(DEVICE_PROGRAM_SLOT), ClearSlotResponse)
        if not clear_response.success:
            print("Failed to clear program slot. This could mean it is already empty, proceeding...")

        program_crc = crc(PROGRAM_TO_SEND)
        start_upload_response = await send_request(
            StartFileUploadRequest("program1.py", DEVICE_PROGRAM_SLOT, program_crc),
            StartFileUploadResponse
        )
        if not start_upload_response.success:
            print("Error: File upload failed.")
            sys.exit(1)

        running_crc = 0
        for i in range(0, len(PROGRAM_TO_SEND), info_response.max_chunk_size):
            chunk = PROGRAM_TO_SEND[i : i + info_response.max_chunk_size]
            running_crc = crc(chunk, running_crc)
            chunk_response = await send_request(TransferChunkRequest(running_crc, chunk), TransferChunkResponse)
            if not chunk_response.success:
                print(f"Error: failed to transfer chunk {i}")
                sys.exit(1)

        start_program_response = await send_request(
            ProgramFlowRequest(stop=False, slot=DEVICE_PROGRAM_SLOT), ProgramFlowResponse
        )
        if not start_program_response.success:
            print("Error: failed to start program")
            sys.exit(1)

        '''
        Waiting for the program to terminate or device disconnect
        '''

        await stop_event.wait()
        await client.stop_notify(tx)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user.")
        stop_event.set()