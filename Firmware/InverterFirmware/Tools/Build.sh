#!/bin/bash

# Get Into Root Repo Dir
echo "Entering Root Repo Directory"
cd ..

# Configure Build Type
BuildType="Debug"
if (($# >= 2))
then
    BuildType=$2
    echo "Building in user-specified $BuildType mode."
else
    echo "Did not get specified build configuration, building in $BuildType mode."
fi

# Check If Build Type Correct
if grep -q $BuildType "Build/BuildType"
then
    echo "Detected Matching Build Type"
else
    echo "Build Type Mismatch, Cleaning First"
    rm -rf Build/
fi

# Check If Configuration Needs To Be Run
echo "Checking If Build Directory Already Exists"
if [ -d "Build" ]
then
    echo "Build Directory Already Exists, Skipping Generation"
    cd Build
else
    # Create Build Dir
    echo "Creating Build Directory"
    mkdir -p Build

    # Enter Build Dir
    echo "Entering Build Directory"
    cd Build

    # Make Only BrainGenix-NES
    echo "Configuring Build Files"
    cmake .. -D CMAKE_BUILD_TYPE=$BuildType

    # Set Config Var
    echo "Saving Build Type Configuration Of $BuildType"
    echo "$BuildType" > "BuildType"
fi

# Build Files
echo "Building, Please Wait. This may take some time"
cmake --build . -j $1

# Return to Tools directory
cd ../Tools
