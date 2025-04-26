#!/bin/bash
# Build and upload an Arduino sketch using arduino-cli
# Usage: ./build.sh <SketchName> [Port]

SUPPORTED_SKETCHES=("ClientA" "ClientB" "BoostConverter" "BoostConverter_Test" "DynamicBrakeGrid" "DynamicBrakeGrid_Test")

SKETCH_NAME="$1"
PORT="${2:-/dev/ttyUSB0}"
BOARD="arduino:avr:uno"
PROJECT_DIR="$(pwd)"
SKETCH_DIR="${PROJECT_DIR}/${SKETCH_NAME}"
SKETCH_FILE="${SKETCH_DIR}/${SKETCH_NAME}.ino"
LIBRARY_DIR="${PROJECT_DIR}/CANFramework"

# Function to print a divider
divider() {
  echo "------------------------------------------------------------"
}

# Check if sketch name was provided
if [ -z "$SKETCH_NAME" ]; then
    echo "Usage: ./build.sh <SketchName> [Port]"
    echo "Example: ./build.sh ClientA /dev/ttyUSB1"
    exit 1
fi

# Validate sketch name
if [[ ! " ${SUPPORTED_SKETCHES[@]} " =~ " ${SKETCH_NAME} " ]]; then
    echo "Error: Unsupported sketch name '${SKETCH_NAME}'."
    echo "Supported sketches: ${SUPPORTED_SKETCHES[*]}"
    exit 1
fi

# Check if sketch file exists
if [ ! -f "$SKETCH_FILE" ]; then
    echo "Error: Sketch file not found at ${SKETCH_FILE}"
    exit 1
fi

divider
echo "Compiling sketch: ${SKETCH_NAME}"
echo "Using board: ${BOARD}"
echo "Sketch path: ${SKETCH_FILE}"
divider

arduino-cli compile \
    --fqbn $BOARD \
    --libraries "$LIBRARY_DIR" \
    "$SKETCH_DIR"

if [ $? -ne 0 ]; then
    echo "Build failed. Aborting upload."
    exit 1
fi

divider
echo "Uploading to port: $PORT"
divider

arduino-cli upload \
    -p "$PORT" \
    --fqbn $BOARD \
    "$SKETCH_DIR"

echo "✅ Done."
