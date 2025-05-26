#!/bin/sh

add_line_to_bashrc()
{
  line_to_add=$1

  echo "line_to_add: $line_to_add"

  # Check if the line is already present in .bashrc
  if ! grep -qF "$line_to_add" ~/.bashrc; then
    # If not present, add the line to the end of .bashrc
    echo "$line_to_add" >> ~/.bashrc
    echo "Line added to .bashrc"
  else
    echo "Line already exists in .bashrc. Nothing to do."
  fi
}

#working_directory=$(pwd)
working_directory=~/git
if [ ! -d "$working_directory" ]; then
  echo "destination does not exist, creating directory: $working_directory"
  mkdir $working_directory
fi

cd $working_directory

echo "first we need to update."
sudo apt update
sudo apt upgrade

echo "now we need to install requried packages."
sudo apt install build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

echo "cloning pico-sdk"
git clone https://github.com/raspberrypi/pico-sdk.git
echo "adding pico-sdk to path"
add_line_to_bashrc "export PICO_SDK_PATH=$working_directory/pico-sdk"

echo "cloning pico-extras"
git clone https://github.com/raspberrypi/pico-extras.git
echo "adding pico-extras to path"
add_line_to_bashrc "export PICO_EXTRAS_PATH=$working_directory/pico-extras"

echo "cloning pico-examples"
git clone https://github.com/raspberrypi/pico-examples.git

echo "updating pico-sdk"
cd pico-sdk
git submodule update --init --recursive
cd ..

echo "updating pico-extras"
cd pico-extras
git submodule update --init --recursive
cd ..

echo finished!

echo "You need to run the following command:"
echo "'source ~/.bashrc'"
echo "To update the path variables on any open terminals."
