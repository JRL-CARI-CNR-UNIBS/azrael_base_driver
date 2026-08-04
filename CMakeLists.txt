cmake_minimum_required(VERSION 3.10)
project(azrael_mobile_driver)

find_package(iir REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Compile as C++17
add_compile_options(-std=c++17)
#add_compile_definitions(DEBUG_PID)

include_directories(include)

add_executable(${PROJECT_NAME} src/driver.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -mcpu=cortex-a72 -mtune=cortex-a72 -O3)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    wiringPi
    pthread
    phidget22
    rt
    iir
    Boost::system
)

