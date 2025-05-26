#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import threading
import time
import sys

class SerialMonitor:
    def __init__(self):
        self.serial_port = None
        self.running = False
        self.baudrate = 115200  # Common baud rate for Pico
        self.default_port = "/dev/ttyACM0"

    def find_pico_port(self):
        """Attempt to automatically find the Pico's serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Raspberry Pi Pico" in port.description or "USB Serial Device" in port.description:
                return port.device
        return self.default_port

    def start_serial(self, port=None):
        """Initialize serial connection"""
        if not port:
            port = self.find_pico_port()
            print(f"Auto-detected port: {port}")

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Wait for connection to establish
            self.running = True
            print(f"Connected to {port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return False

    def read_serial(self):
        """Read data from serial port"""
        while self.running and self.serial_port:
            if self.serial_port.in_waiting > 0:
                try:
                    line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        print(f"<< {line}")
                except:
                    pass

    def write_serial(self):
        """Send data to serial port"""
        print("\nEnter commands to send (type 'exit' to quit):")
        while self.running and self.serial_port:
            try:
                cmd = input(">> ").strip()
                if cmd.lower() == 'exit':
                    self.stop()
                    break
                if cmd:
                    self.serial_port.write((cmd + '\n').encode('utf-8'))
            except KeyboardInterrupt:
                self.stop()
                break
            except:
                pass

    def stop(self):
        """Cleanup serial connection"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        print("\nSerial connection closed")

    def run(self):
        """Main execution"""
        print("Raspberry Pi Pico Serial Monitor")
        print("--------------------------------")
        
        # Try to auto-detect port
        if not self.start_serial():
            # If auto-detection fails, prompt for manual port
            port = input(f"Enter serial port [{self.default_port}]: ").strip()
            if not port:
                port = self.default_port
            if not self.start_serial(port):
                sys.exit(1)

        # Start serial read thread
        read_thread = threading.Thread(target=self.read_serial, daemon=True)
        read_thread.start()

        # Run write in main thread
        self.write_serial()

if __name__ == "__main__":
    monitor = SerialMonitor()
    monitor.run()