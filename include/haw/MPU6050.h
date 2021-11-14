/**
 * InvenSense MPU6050 3-axis accelerometer and 3-axis gyroscope sensor stand-alone library for pico-sdk.
 * 
 * This library eases the use of the MPU6050 by hiding calculations and comparisons, while still maintaining
 * a flexible setup. My libraries have usually the following patterns -> Initialization, Setup and in the program
 * loop, only one event call, so I2C is only used at one place in your program. See the example file for more
 * information.
 * 
 * Version History:
 * 1.0 -- Initial release
 * 
 * @file MPU6050.h
 * @author Maik Steiger (maik.steiger@tu-dortmund.de)
 * @brief 
 * @version 1.0
 * @date 2021-11-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef MPU6050_H_
#define MPU6050_H_
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_ADDRESS_A0_VCC 0x69
#define MPU6050_ADDRESS_A0_GND 0x68

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef I2C_INFORMATION_S_
#define I2C_INFORMATION_S_
    struct i2c_information
    {
        i2c_inst_t *instance;
        uint8_t address;
    };
#endif

    struct mpu6050_vector16
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    typedef struct mpu6050_vectorf
    {
        float x;
        float y;
        float z;
    } mpu6050_vectorf_t;

    struct mpu6050_configuration
    {
        uint8_t use_calibrate;
        uint8_t actual_threshold;
        float dps_per_digit;
        float range_per_digit;
        uint8_t meas_temp;
        uint8_t meas_acce;
        uint8_t meas_gyro;
        uint8_t dhpf;
        uint8_t dlpf;
    };

    struct mpu6050_calibration_data
    {
        struct mpu6050_vectorf tg; // Theshold for gyroscope
        struct mpu6050_vectorf dg; // Delta for gyroscope
        struct mpu6050_vectorf th; // Threshold
    };

    enum MPU6050_CLOCK_SOURCE
    {
        MPU6050_CLOCK_INTERNAL = 0,
        MPU6050_CLOCK_PLL_XGYRO = 1,
        MPU6050_CLOCK_PLL_YGYRO = 2,
        MPU6050_CLOCK_PLL_ZGYRO = 3,
        MPU6050_CLOCK_EXTERNAL_32KHZ = 4,
        MPU6050_CLOCK_EXTERNAL_19MHZ = 5,
        MPU6050_CLOCK_KEEP_RESET = 7,
    };

    enum MPU6050_SCALE
    {
        MPU6050_SCALE_250DPS = 0,
        MPU6050_SCALE_500DPS = 1,
        MPU6050_SCALE_1000DPS = 2,
        MPU6050_SCALE_2000DPS = 3
    };

    enum MPU6050_RANGE
    {
        MPU6050_RANGE_2G = 0,
        MPU6050_RANGE_4G = 1,
        MPU6050_RANGE_8G = 2,
        MPU6050_RANGE_16G = 3,
    };

    enum MPU6050_DHPF
    {
        MPU6050_DHPF_RESET = 0,
        MPU6050_DHPF_5HZ = 1,
        MPU6050_DHPF_2_5HZ = 2,
        MPU6050_DHPF_1_25HZ = 3,
        MPU6050_DHPF_0_63HZ = 4,
        MPU6050_DHPF_HOLD = 7
    };

    enum MPU6050_DLPF
    {
        MPU6050_DLPF_0 = 0,
        MPU6050_DLPF_1 = 1,
        MPU6050_DLPF_2 = 2,
        MPU6050_DLPF_3 = 3,
        MPU6050_DLPF_4 = 4,
        MPU6050_DLPF_5 = 5,
        MPU6050_DLPF_6 = 6,
    };

    /**
     * @brief MPU6050 Interrupt flags struct. It stores all the interrupt flags internally, which can be read and checked if a condition is met.
     * These flags are reset automatically by the sensor itself, so READ-ONLY.
     */
    typedef struct mpu6050_activity
    {
        uint8_t isOverflow;
        uint8_t isFreefall;
        uint8_t isInactivity;
        uint8_t isActivity;
        uint8_t isPosActivityOnX;
        uint8_t isPosActivityOnY;
        uint8_t isPosActivityOnZ;
        uint8_t isNegActivityOnX;
        uint8_t isNegActivityOnY;
        uint8_t isNegActivityOnZ;
        uint8_t isDataReady;
    } mpu6050_activity_t;

    /**
     * @brief Main MPU6050 struct. It holds all the needed informations to make the MPU6050 work correcly. 
     * Never change any attributes directly, do so only by calling their appropriate functions. 
     */
    typedef struct mpu6050
    {
        struct i2c_information i2c;
        struct mpu6050_configuration config;
        struct mpu6050_calibration_data calibration_data;
        struct mpu6050_activity activity;
        struct mpu6050_vector16 ra; // Raw accelerometer vector
        struct mpu6050_vector16 rg; // Raw gyroscope vector
        struct mpu6050_vectorf na;  // Normalized accelerometer vector
        struct mpu6050_vectorf ng;  // Normalized gyroscope vector
        struct mpu6050_vectorf sa;  // Scaled accelerometer vector
        int16_t raw_temperature;
        float temperature;
        float temperaturef;
    } mpu6050_t;

    /**
     * @brief Initialized a MPU6050 struct and returns it.
     * 
     * @param i2c_inst_t* i2c_instance: I2C bus instance. Needed for multicore applications. 
     * @param uint8_t address: Slave device address, which can be modified by tying pin A0 either to GND or VCC.
     * @return struct mpu6050 
     */
    struct mpu6050 mpu6050_init(i2c_inst_t *i2c_instance, const uint8_t address);

    /**
     * @brief Checks if MPU6050 is connected to the bus and runs a default setup.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return uint8_t: 1 if setup is successful, otherwise 0
     */
    uint8_t mpu6050_begin(struct mpu6050 *self);

    /**
     * @brief Sets the scale of gyroscope readings. The higher, the preciser but slower.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param MPU6050_SCALE scale: Scale of gyroscope readings 
     */
    void mpu6050_set_scale(struct mpu6050 *self, enum MPU6050_SCALE scale);

    /**
     * @brief Sets the range of accelerometer readings. The higher, the preciser but slower.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param MPU6050_RANGE range: Range of accelerometer readings 
     */
    void mpu6050_set_range(struct mpu6050 *self, enum MPU6050_RANGE range);

    /**
     * @brief Sets the clock input for the MPU6050. If not specified, it automatically uses the internal 8MHz oscillator.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param MPU6050_CLOCK_SOURCE clock_source: Source of the clock signal 
     */
    void mpu6050_set_clock_source(struct mpu6050 *self, enum MPU6050_CLOCK_SOURCE clock_source);

    /**
     * @brief Puts the MPU6050 into sleep mode if set to 0. If set to 1, the device awakes.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: State to put the device in
     */
    void mpu6050_set_sleep_enabled(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Returns the WHO_AM_I identification of the device, which is always by default 0x68.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return uint8_t: WHO_AM_I identification (0x68)
     */
    uint8_t mpu6050_who_am_i(struct mpu6050 *self);

    /**
     * @brief Fetches all the data from the device. The results are getting store into their buffers.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return uint8_t: 1 if readings were successful, otherwise 0
     */
    uint8_t mpu6050_event(struct mpu6050 *self);

    /**
     * @brief Enables or disables temperature measurement readings. If state is set to 1, then
     * the temperature will be read after an event call. Otherwise temperature readings will be
     * skipped. Keep in mind however, that this is not a temperature sensor. So the resulted
     * temperatures might be not precise.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: 1 to enable or 0 to disable
     */
    void mpu6050_set_temperature_measuring(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Enables or disables accelerometer measurement readings. If state is set to 1, then
     * the accelerometer will be read after an event call. Otherwise accelerometer readings will be
     * skipped.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: 1 to enable or 0 to disable
     */
    void mpu6050_set_accelerometer_measuring(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Enables or disables gyroscope measurement readings. If state is set to 1, then
     * the gyroscope will be read after an event call. Otherwise gyroscope readings will be
     * skipped.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: 1 to enable or 0 to disable
     */
    void mpu6050_set_gyroscope_measuring(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Calibrates the gyroscope by taking n amounts of samples and
     * average them out to later be used in gyroscope calculations.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t samples: Amount of samples to take (Default 50 if you are unsure)
     */
    void mpu6050_calibrate_gyro(struct mpu6050 *self, uint8_t samples);

    /**
     * @brief Sets the threshold value of the device, which are stored
     * as calibration values.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t multiple: Treshold value (Default 1 if you are unsure)
     */
    void mpu6050_set_threshold(struct mpu6050 *self, uint8_t multiple);

    /**
     * @brief Sets the gyroscope x-axis offset.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint16_t offset: Offset amount 
     */
    void mpu6050_set_gyro_offset_x(struct mpu6050 *self, uint16_t offset);

    /**
     * @brief Sets the gyroscope y-axis offset.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint16_t offset: Offset amount 
     */
    void mpu6050_set_gyro_offset_y(struct mpu6050 *self, uint16_t offset);

    /**
     * @brief Sets the gyroscope z-axis offset.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint16_t offset: Offset amount 
     */
    void mpu6050_set_gyro_offset_z(struct mpu6050 *self, uint16_t offset);

    /**
     * @brief Sets the accelerometer x-axis offset.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint16_t offset: Offset amount 
     */
    void mpu6050_set_accel_offset_x(struct mpu6050 *self, uint16_t offset);

    /**
     * @brief Sets the accelerometer y-axis offset.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint16_t offset: Offset amount 
     */
    void mpu6050_set_accel_offset_y(struct mpu6050 *self, uint16_t offset);

    /**
     * @brief Sets the accelerometer z-axis offset.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint16_t offset: Offset amount 
     */
    void mpu6050_set_accel_offset_z(struct mpu6050 *self, uint16_t offset);

    /**
     * @brief Calculates and returns a pointer to the accelerometer data.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return mpu6050_vectorf_t*: Pointer to float vector of accelerometer data 
     */
    struct mpu6050_vectorf *mpu6050_get_accelerometer(struct mpu6050 *self);

    /**
     * @brief Calculates and returns a pointer to the accelerometer data. The calculations will
     * ignore the relative gravitation to earth.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return mpu6050_vectorf_t*: Pointer to float vector of accelerometer data 
     */
    struct mpu6050_vectorf *mpu6050_get_scaled_accelerometer(struct mpu6050 *self);

    /**
     * @brief Calculates and returns a pointer to the gyroscope data.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return mpu6050_vectorf_t*: Pointer to float vector of gyroscope data 
     */
    struct mpu6050_vectorf *mpu6050_get_gyroscope(struct mpu6050 *self);

    /**
     * @brief Calculates and returns the temperature in celsius.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return float: Temperature in celsius
     */
    float mpu6050_get_temperature_c(struct mpu6050 *self);

    /**
     * @brief Calculates and returns the temperature in fahrenheit.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return float: Temperature in fahrenheit
     */
    float mpu6050_get_temperature_f(struct mpu6050 *self);

    /**
     * @brief Returns a pointer to the activity struct which contains all the
     * interrupt flags, which are reset automatically the the device. That means READ-ONLY.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @return mpu6050_activity_t*: Pointer to activity struct 
     */
    struct mpu6050_activity *mpu6050_read_activities(struct mpu6050 *self);

    /**
     * @brief Enables or disables interrupt flags for motion detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: 1 to enable or 0 to disable 
     */
    void mpu6050_set_int_motion(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Enables or disables interrupt flags for zero motion detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: 1 to enable or 0 to disable 
     */
    void mpu6050_set_int_zero_motion(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Enables or disables interrupt flags for free fall detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t state: 1 to enable or 0 to disable 
     */
    void mpu6050_set_int_free_fall(struct mpu6050 *self, uint8_t state);

    /**
     * @brief Sets the DHPF (Digital High Pass Filter)
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param MPU6050_DHPF dhpf 
     */
    void mpu6050_set_dhpf_mode(struct mpu6050 *self, enum MPU6050_DHPF dhpf);

    /**
     * @brief Sets the DLPF (Digital Low Pass Filter)
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param MPU6050_DLPF dlpf 
     */
    void mpu6050_set_dlpf_mode(struct mpu6050 *self, enum MPU6050_DLPF dlpf);

    /**
     * @brief Sets the threshold for motion detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t threshold: Threshold value 
     */
    void mpu6050_set_motion_detection_threshold(struct mpu6050 *self, uint8_t threshold);

    /**
     * @brief Sets the duration for motion detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t duration: Duration value
     */
    void mpu6050_set_motion_detection_duration(struct mpu6050 *self, uint8_t duration);

    /**
     * @brief Sets the threshold for zero motion detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t threshold: Threshold value 
     */
    void mpu6050_set_zero_motion_detection_threshold(struct mpu6050 *self, uint8_t threshold);

    /**
     * @brief Sets the duration for zero motion detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t duration: Duration value
     */
    void mpu6050_set_zero_motion_detection_duration(struct mpu6050 *self, uint8_t duration);

    /**
     * @brief Sets the threshold for free fall detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t threshold: Threshold value 
     */
    void mpu6050_set_free_fall_detection_threshold(struct mpu6050 *self, uint8_t threshold);

    /**
     * @brief Sets the duration for free fall detection.
     * 
     * @param mpu6050_t* self: Reference to itself 
     * @param uint8_t duration: Duration value
     */
    void mpu6050_set_free_fall_detection_duration(struct mpu6050 *self, uint8_t duration);

#ifdef __cplusplus
}
#endif
#endif