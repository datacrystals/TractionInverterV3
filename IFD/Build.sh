#!/bin/bash

mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH="path/to/your/Qt5/installation"  # Only if Qt is not in default path
cmake --build .