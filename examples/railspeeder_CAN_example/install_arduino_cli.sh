#!/bin/bash

set -e

# === Config ===
CLI_PATH="/usr/local/bin/arduino-cli"

# === Functions ===
is_installed() {
    command -v arduino-cli &>/dev/null
}

has_avr_core() {
    arduino-cli core list | grep -q "arduino:avr"
}

# === Main ===

echo "🔍 Checking if Arduino CLI is installed..."
if is_installed; then
    echo "✅ Arduino CLI is already installed at $(which arduino-cli)"
    echo "    Version: $(arduino-cli version)"
else
    echo "📦 Installing Arduino CLI..."
    cd /tmp
    curl -fsSL https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz -o arduino-cli.tar.gz
    tar -xzf arduino-cli.tar.gz

    echo "🚚 Moving CLI to $CLI_PATH (requires sudo)..."
    sudo mv arduino-cli "$CLI_PATH"

    echo "✅ Arduino CLI installed at $CLI_PATH"
fi

# Config init (if needed)
CONFIG_PATH="$HOME/.arduino15/arduino-cli.yaml"
if [ -f "$CONFIG_PATH" ]; then
    echo "⚙️ Arduino CLI config already exists at $CONFIG_PATH"
else
    echo "🛠️ Initializing Arduino CLI config..."
    arduino-cli config init
fi

# Install AVR core
if has_avr_core; then
    echo "✅ Core 'arduino:avr' already installed."
else
    echo "📥 Installing AVR core..."
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

echo "🎉 Setup complete!"
