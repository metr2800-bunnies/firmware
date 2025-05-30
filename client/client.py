#!/usr/bin/env python3
# this script is a BLE telemetry client that records data from the robot to a CSV file
import asyncio
import struct
from bleak import BleakClient, BleakScanner
import os

# UUIDs from ESP32S3 NimBLE code
SERVICE_UUID = "1234"
TELEMETRY_CHAR_UUID = "8765"

async def find_device(timeout=10.0, retries=3):
    for attempt in range(retries):
        print(f"Scanning for BLE device (attempt {attempt + 1}/{retries})...")
        devices = await BleakScanner.discover(timeout=timeout)
        for device in devices:
            if device.name == "nimble-robot":
                print(f"Found device: {device.name} ({device.address})")
                return device
        print("Device not found, retrying...")
        await asyncio.sleep(1)
    return None

async def main():
    device = await find_device()
    if not device:
        print("Failed to find nimble-robot after retries")
        return

    for attempt in range(3):
        try:
            async with BleakClient(device.address, timeout=20.0) as client:
                print(f"Connected to {device.name}")

                # Log available services and characteristics
                services = client.services
                print("Available services and characteristics:")
                if not services or not services.services:
                    print("  No services found!")
                for service in services:
                    print(f"Service: {service.uuid}")
                    for char in service.characteristics:
                        print(f"  Characteristic: {char.uuid}, Properties: {char.properties}")

                def telemetry_handler(sender, data):
                    try:
                        unpacked = struct.unpack_from("<I4i4f4f4f", data, 0)
                        packet_num = unpacked[0]
                        encoder = unpacked[1:5]
                        rpm = unpacked[5:9]
                        drive_power = unpacked[9:13]
                        target_rpms = unpacked[13:17]

                        write_headers = not os.path.exists('data.csv')
                        with open('data.csv', 'a') as file:
                            if write_headers:
                                headers = (
                                    ["packet_num"] +
                                    [f'encoder_count_{i}' for i in range(4)] +
                                    [f'rpm_{i}' for i in range(4)] +
                                    [f'drive_power{i}' for i in range(4)] +
                                    [f'target_rpm_{i}' for i in range(4)]
                                )
                                file.write(','.join(headers))
                                file.write('\n')
                            file.write(','.join(map(str, unpacked)))
                            file.write('\n')
                    except struct.error as e:
                        print(f"Error unpacking telemetry data: {e}")

                # Try subscribing with both UUID cases
                for uuid in [TELEMETRY_CHAR_UUID]:
                    try:
                        print(f"Trying to subscribe to characteristic: {uuid}")
                        await client.start_notify(uuid, telemetry_handler)
                        print("Subscribed to telemetry notifications")
                        break
                    except Exception as e:
                        print(f"Failed to subscribe to {uuid}: {e}")
                else:
                    print("Failed to subscribe with both UUID cases")
                    return

                try:
                    while True:
                        await asyncio.sleep(1)
                except KeyboardInterrupt:
                    print("Disconnecting...")
                    await client.stop_notify(uuid)
            break
        except Exception as e:
            print(f"Connection attempt {attempt + 1}/3 failed: {e}")
            await asyncio.sleep(1)
    else:
        print("Failed to connect after retries")

if __name__ == "__main__":
    asyncio.run(main())
