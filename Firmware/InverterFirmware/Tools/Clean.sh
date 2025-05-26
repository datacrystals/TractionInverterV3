#!/bin/bash

# Navigate to the root directory (assuming the script is run from a subdirectory)
cd ..

# Define the Build directory path
BUILD_DIR="Build"

# Check if the Build directory exists
if [ -d "$BUILD_DIR" ]; then
    echo "Found Build directory. Removing..."
    # Remove the Build directory and all its contents
    rm -rf "$BUILD_DIR"
    echo "Build directory removed successfully."
else
    echo "Build directory not found. Nothing to remove."
fi
