#!/bin/bash
# Initialize arduino-cli configuration and install required core and libraries

echo "Initializing Arduino CLI..."
arduino-cli config init

echo "Installing Arduino AVR core..."
arduino-cli core install arduino:avr

echo "Installing MCP_CAN library..."
arduino-cli lib install "mcp_can"
arduino-cli lib install "mcp2515"
# Clone the mcp2515 library into the Arduino libraries directory
git clone https://github.com/autowp/arduino-mcp2515.git ~/Arduino/libraries/mcp2515

echo "Setup complete."
