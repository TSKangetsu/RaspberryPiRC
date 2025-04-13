cmake_minimum_required (VERSION 3.8)
add_definitions(-DDEBUG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
project ("RaspberryPiRC")

add_subdirectory(RPiGPS)
add_subdirectory(RPiIBus)
add_subdirectory(RPiSBus)
add_subdirectory(RPiFlow)
add_subdirectory(CRSF)

set(CMAKE_EXE_LINKER_FLAGS "-static")

add_executable (RaspberryPiRC "TestModule.cpp")
target_link_libraries(RaspberryPiRC ${Ibus})
target_link_libraries(RaspberryPiRC ${Sbus})
target_link_libraries(RaspberryPiRC ${GPSUart})
target_link_libraries(RaspberryPiRC ${RPiFlow})
target_link_libraries(RaspberryPiRC ${LibCRSF})

target_link_libraries(RaspberryPiRC gcc stdc++ gcc_eh)
