# Create an executable
add_executable(mpu6050_accelerometer_and_gyro_read 
    mpu6050_accelerometer_and_gyro_read.c
)

# Link all required libraries
target_link_libraries(mpu6050_accelerometer_and_gyro_read 
    pico_stdlib
    hardware_i2c 
    haw_mpu6050
)

# Enable printf output for USB communication
pico_enable_stdio_usb(mpu6050_accelerometer_and_gyro_read 1)
pico_enable_stdio_uart(mpu6050_accelerometer_and_gyro_read 0)

# Add the .uf2 output to the build directory
pico_add_extra_outputs(mpu6050_accelerometer_and_gyro_read)