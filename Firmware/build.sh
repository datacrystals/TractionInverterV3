#!/bin/bash
# Build and upload ClientA or ClientB to an Arduino board using arduino-cli

if [ "$#" -ne 1 ]; then
    echo "Usage: ./build.sh [ClientA|ClientB]"
    exit 1
fi

SKETCH_NAME="$1"
BOARD="arduino:avr:uno"
PORT="/dev/ttyUSB0"  # Adjust to match your actual serial port

SKETCH_DIR="$(pwd)/${SKETCH_NAME}"
LIBRARY_DIR="$(pwd)/CANFramework"
SKETCH_FILE="${SKETCH_DIR}/${SKETCH_NAME}.ino"

if [ ! -f "${SKETCH_DIR}/${SKETCH_NAME}.ino" ]; then
    echo "Sketch file not found: ${SKETCH_DIR}/${SKETCH_NAME}.ino"
    exit 1
fi

echo "Compiling ${SKETCH_NAME}.ino..."
arduino-cli compile \
    --fqbn $BOARD \
    --libraries "$LIBRARY_DIR" \
    "$SKETCH_DIR"

if [ $? -ne 0 ]; then
    echo "Build failed. Aborting upload."
    exit 1
fi

echo "Uploading to $PORT..."
arduino-cli upload -p "$PORT" --fqbn $BOARD "$SKETCH_DIR"

echo "Done."
