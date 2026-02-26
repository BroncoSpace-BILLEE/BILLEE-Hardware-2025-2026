#!/usr/bin/env python3

import serial
import threading
import sys

PORT = "/dev/ttyACM0"   # Change if needed
BAUD = 115200

def read_from_port(ser):
    """Continuously read from serial and print to console."""
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode(errors='ignore')
                print(data, end='', flush=True)
        except:
            break

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f"Connected to {PORT} at {BAUD} baud.")
        print("Press Ctrl+C to exit.\n")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # Start background thread to read serial data
    reader = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
    reader.start()

    try:
        while True:
            user_input = input()
            ser.write((user_input + "\n").encode())
    except KeyboardInterrupt:
        print("\nClosing connection...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()