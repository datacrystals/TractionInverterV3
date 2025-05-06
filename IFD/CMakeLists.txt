cmake_minimum_required(VERSION 3.16)
project(TrainHMI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(TrainHMI
    Main.cpp
    TitledValueDisplay.h
    VerticalBarGauge.h
)

target_link_libraries(TrainHMI
    Qt5::Widgets
)