# Include source file(s) to project
add_library(haw_mpu6050 STATIC
    MPU6050.c
)

# Links all required dependencies
target_link_libraries(haw_mpu6050 
    pico_stdlib
    hardware_i2c 
)

# Link include directory / Path to header file
target_include_directories(haw_mpu6050 PUBLIC ../include)