cmake_minimum_required(VERSION 3.5)
project(TrainHMI VERSION 1.0 LANGUAGES CXX)

# Find required Qt components
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Add executable
add_executable(TrainHMI
    Main.cpp
)

# Link Qt libraries
target_link_libraries(TrainHMI
    Qt5::Widgets
)

# Set C++ standard and properties
set_target_properties(TrainHMI PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    WIN32_EXECUTABLE ON
)