cmake_minimum_required (VERSION 3.8)

project ("RaspberryPiRC")
find_library(WIRING wiringPi)

add_subdirectory(RPiIBus)
add_subdirectory(RPiSBus)

add_executable (RaspberryPiRC "TestModule.cpp")
target_link_libraries(RaspberryPiRC  ${WIRING})
target_link_libraries(RaspberryPiRC ${Ibus})
target_link_libraries(RaspberryPiRC ${Sbus})
