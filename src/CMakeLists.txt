# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(    
    SRCS
        "main.c"
        "app_state.c"
        "adc_current.c"
        "boost_pwm.c"
        "buttons.c"
        "current_ctrl.c"
        "font8x8_basic.c"
        "pwm12.c"
        "ssd1306.c"
        "ui_oled.c"
    INCLUDE_DIRS
        "."
        "../include"
)
