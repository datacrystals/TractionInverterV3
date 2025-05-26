#!/usr/bin/env python3
import serial
import sys
import select
import time

# Configuration
PORT = "/dev/ttyACM0"  # Change this if your Pico uses a different port
BAUDRATE = 115200      # Standard baud rate for Pico
RECONNECT_DELAY = 0.5    # Seconds between connection attempts

class PersistentSerialMonitor:
    def __init__(self):
        self.running = True
        self.ser = None

    def connect(self):
        """Continuously try to establish serial connection"""
        while self.running:
            try:
                # print(f"Attempting to connect to {PORT}...")
                self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
                print(f"Connected to {PORT} at {BAUDRATE} baud")
                print("Type commands to send to the Pico (Ctrl+C to exit):")
                return True
            except serial.SerialException as e:
                # print(f"Connection failed: {e}")
                # print(f"Retrying in {RECONNECT_DELAY} seconds...")
                time.sleep(RECONNECT_DELAY)
        return False

    def monitor(self):
        """Main monitoring loop"""
        while self.running:
            if not self.ser or not self.ser.is_open:
                if not self.connect():
                    continue
            
            try:
                # Check for incoming serial data
                if self.ser.in_waiting > 0:
                    print(self.ser.readline().decode('utf-8', errors='replace').strip())
                
                # Check for user input (non-blocking)
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    cmd = sys.stdin.readline().strip()
                    if cmd.lower() in ('exit', 'quit'):
                        self.running = False
                        break
                    try:
                        self.ser.write((cmd + '\n').encode('utf-8'))
                    except serial.SerialException:
                        print("Write failed - connection may be lost")
                        self.ser.close()
                        continue
                        
            except serial.SerialException:
                print("Connection lost - attempting to reconnect...")
                self.ser.close()
                time.sleep(RECONNECT_DELAY)
                continue
            except KeyboardInterrupt:
                self.running = False
                break

    def cleanup(self):
        """Clean up resources"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("\nSerial monitor stopped")

if __name__ == "__main__":
    monitor = PersistentSerialMonitor()
    try:
        monitor.monitor()
    finally:
        monitor.cleanup()