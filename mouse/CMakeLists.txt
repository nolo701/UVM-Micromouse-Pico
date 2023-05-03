

add_library(mouse
        define.h
        )

target_link_libraries(mouse
        hardware_i2c
        hardware_pwm
        pico_stdlib
        )
        
target_include_directories (mouse PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
