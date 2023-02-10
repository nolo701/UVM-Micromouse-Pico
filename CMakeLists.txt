# made from : https://www.youtube.com/watch?v=3Uy9ySoMXAY

# select the GCC 10.3.1 arm-none-eabi for the kit then build
# drag the uf2 into the pi's folder.
# hold bootsel button while plugging in to get folder to show up
cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)

project(PicoMouse_project)

pico_sdk_init()
add_subdirectory(pico-ssd1306)
add_subdirectory(mouse)
include(${PROJECT_SOURCE_DIR}/VL53L1X/library/import.cmake)
add_executable(PicoMouse main.cpp)

target_link_libraries(PicoMouse 
    hardware_i2c
    hardware_pwm 
    pico_ssd1306
    VL53L1X
    
    )

#target_link_libraries(sensorSetup pico_stdlib hardware_i2c pico_ssd1306)

pico_add_extra_outputs(PicoMouse)