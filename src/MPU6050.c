#include "haw/MPU6050.h"
#include "math.h"
#include "stdlib.h"

#define ACCEL_XOFFS_H 0x06
#define ACCEL_XOFFS_L 0x07
#define ACCEL_YOFFS_H 0x08
#define ACCEL_YOFFS_L 0x09
#define ACCEL_ZOFFS_H 0x0A
#define ACCEL_ZOFFS_L 0x0B
#define GYRO_XOFFS_H 0x13
#define GYRO_XOFFS_L 0x14
#define GYRO_YOFFS_H 0x15
#define GYRO_YOFFS_L 0x16
#define GYRO_ZOFFS_H 0x17
#define GYRO_ZOFFS_L 0x18
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B  // Gyroscope Configuration
#define ACCEL_CONFIG 0x1C // Accelerometer Configuration
#define FF_THRESHOLD 0x1D
#define FF_DURATION 0x1E
#define MOT_THRESHOLD 0x1F
#define MOT_DURATION 0x20
#define ZMOT_THRESHOLD 0x21
#define ZMOT_DURATION 0x22
#define INT_PIN_CFG 0x37 // INT Pin. Bypass Enable Configuration
#define INT_ENABLE 0x38  // INT Enable
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define MOT_DETECT_STATUS 0x61
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A  // User Control
#define PWR_MGMT_1 0x6B // Power Management 1
#define WHO_AM_I 0x75   // Who Am I

#define GRAVITY_CONSTANT 9.80665f

int i2c_read_reg(struct i2c_information *i2c, const uint8_t reg, uint8_t *buf, const size_t len)
{
    i2c_write_blocking(i2c->instance, i2c->address, &reg, 1, true);
    return i2c_read_blocking(i2c->instance, i2c->address, buf, len, false);
}

int i2c_write(struct i2c_information *i2c, const uint8_t data)
{
    return i2c_write_blocking(i2c->instance, i2c->address, &data, 1, false);
}

void read_raw_gyro(struct mpu6050 *self)
{
    uint8_t data[6];
    i2c_read_reg(&self->i2c, GYRO_XOUT_H, data, 6);

    self->rg.x = data[0] << 8 | data[1];
    self->rg.y = data[2] << 8 | data[3];
    self->rg.z = data[4] << 8 | data[5];
}

void read_raw_accel(struct mpu6050 *self)
{
    uint8_t data[6];
    i2c_read_reg(&self->i2c, ACCEL_XOUT_H, data, 6);

    self->ra.x = data[0] << 8 | data[1];
    self->ra.y = data[2] << 8 | data[3];
    self->ra.z = data[4] << 8 | data[5];
}

inline static void i2c_write_u16_inline(struct i2c_information *i2c, uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (value >> 8), (value & 0xFF)};
    i2c_write_blocking(i2c->instance, i2c->address, data, 3, false);
}

inline static void i2c_write_bit_in_reg_inline(struct i2c_information *i2c, uint8_t reg, uint8_t pos, uint8_t state)
{
    uint8_t reg_value;
    i2c_read_reg(i2c, reg, &reg_value, 1);

    if (state)
    {
        reg_value |= (1 << pos);
    }
    else
    {
        reg_value &= ~(1 << pos);
    }

    uint8_t data[2] = {reg, reg_value};
    i2c_write_blocking(i2c->instance, i2c->address, data, 2, false);
}

struct mpu6050 mpu6050_init(i2c_inst_t *i2c_instance, const uint8_t address)
{
    struct mpu6050 mpu6050;
    mpu6050.i2c.instance = i2c_instance;
    mpu6050.i2c.address = address;

    mpu6050.calibration_data.dg.x = 0;
    mpu6050.calibration_data.dg.y = 0;
    mpu6050.calibration_data.dg.z = 0;
    mpu6050.config.use_calibrate = 0;

    mpu6050.calibration_data.tg.x = 0;
    mpu6050.calibration_data.tg.y = 0;
    mpu6050.calibration_data.tg.z = 0;
    mpu6050.config.actual_threshold = 0;

    mpu6050.config.meas_temp = 0;
    mpu6050.config.meas_acce = 0;
    mpu6050.config.meas_gyro = 0;

    return mpu6050;
}

uint8_t mpu6050_begin(struct mpu6050 *self)
{
    if (mpu6050_who_am_i(self) != 0x68) // 0x68 default WHO_AM_I value
    {
        return 0;
    }

    mpu6050_set_clock_source(self, MPU6050_CLOCK_INTERNAL); // Default Clock
    mpu6050_set_range(self, MPU6050_RANGE_4G);              // Default Range
    mpu6050_set_scale(self, MPU6050_SCALE_500DPS);          // Default Scale
    mpu6050_set_sleep_enabled(self, 0);                     // Disable Sleep Mode

    return 1;
}

uint8_t mpu6050_event(struct mpu6050 *self)
{
    self->rg.x = 0;
    self->rg.y = 0;
    self->rg.z = 0;
    self->ra.x = 0;
    self->ra.y = 0;
    self->ra.z = 0;
    self->ng.x = 0.0f;
    self->ng.y = 0.0f;
    self->ng.z = 0.0f;
    self->na.x = 0.0f;
    self->na.y = 0.0f;
    self->na.z = 0.0f;
    self->raw_temperature = 0;
    self->temperature = 0.0f;
    self->temperaturef = 0.0f;

    if (self->config.meas_gyro)
    {
        read_raw_gyro(self);
    }

    if (self->config.meas_acce)
    {
        read_raw_accel(self);
    }

    if (self->config.meas_temp)
    {
        uint8_t data[2];
        i2c_read_reg(&self->i2c, TEMP_OUT_H, data, 2);
        self->raw_temperature = data[0] << 8 | data[1];
    }

    uint8_t int_status;
    i2c_read_reg(&self->i2c, INT_STATUS, &int_status, 1);

    self->activity.isOverflow = ((int_status >> 4) & 1);
    self->activity.isFreefall = ((int_status >> 7) & 1);
    self->activity.isInactivity = ((int_status >> 5) & 1);
    self->activity.isActivity = ((int_status >> 6) & 1);
    self->activity.isDataReady = ((int_status >> 8) & 1);

    uint8_t mot_detect_status;
    i2c_read_reg(&self->i2c, MOT_DETECT_STATUS, &mot_detect_status, 1);

    self->activity.isNegActivityOnX = ((mot_detect_status >> 7) & 1);
    self->activity.isPosActivityOnX = ((mot_detect_status >> 6) & 1);

    self->activity.isNegActivityOnY = ((mot_detect_status >> 5) & 1);
    self->activity.isPosActivityOnY = ((mot_detect_status >> 4) & 1);

    self->activity.isNegActivityOnZ = ((mot_detect_status >> 3) & 1);
    self->activity.isPosActivityOnZ = ((mot_detect_status >> 2) & 1);
}

void mpu6050_set_scale(struct mpu6050 *self, enum MPU6050_SCALE scale)
{
    uint8_t gyro_config;

    switch (scale)
    {
    case MPU6050_SCALE_250DPS:
        self->config.dps_per_digit = .007633f;
        break;
    case MPU6050_SCALE_500DPS:
        self->config.dps_per_digit = .015267f;
        break;
    case MPU6050_SCALE_1000DPS:
        self->config.dps_per_digit = .030487f;
        break;
    case MPU6050_SCALE_2000DPS:
        self->config.dps_per_digit = .060975f;
        break;
    }

    i2c_read_reg(&self->i2c, GYRO_CONFIG, &gyro_config, 1);
    gyro_config &= 0xE7;
    gyro_config |= (scale << 3);

    uint8_t data[2] = {GYRO_CONFIG, gyro_config};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_range(struct mpu6050 *self, enum MPU6050_RANGE range)
{
    uint8_t accel_config;

    switch (range)
    {
    case MPU6050_RANGE_2G:
        self->config.range_per_digit = .000061f;
        break;
    case MPU6050_RANGE_4G:
        self->config.range_per_digit = .000122f;
        break;
    case MPU6050_RANGE_8G:
        self->config.range_per_digit = .000244f;
        break;
    case MPU6050_RANGE_16G:
        self->config.range_per_digit = .0004882f;
        break;
    }

    i2c_read_reg(&self->i2c, ACCEL_CONFIG, &accel_config, 1);
    accel_config &= 0xE7;
    accel_config |= (range << 3);

    uint8_t data[2] = {ACCEL_CONFIG, accel_config};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_clock_source(struct mpu6050 *self, enum MPU6050_CLOCK_SOURCE clock_source)
{
    uint8_t power_managment_1;
    i2c_read_reg(&self->i2c, PWR_MGMT_1, &power_managment_1, 1);
    power_managment_1 &= 0xF8;
    power_managment_1 |= clock_source;

    uint8_t data[2] = {PWR_MGMT_1, power_managment_1};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_sleep_enabled(struct mpu6050 *self, uint8_t state)
{
    uint8_t data[2] = {PWR_MGMT_1};
    if (state)
    {
        data[1] = 1;
    }
    else
    {
        data[1] = 0;
    }

    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

uint8_t mpu6050_who_am_i(struct mpu6050 *self)
{
    uint8_t who_am_i;
    i2c_read_reg(&self->i2c, WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

void mpu6050_set_temperature_measuring(struct mpu6050 *self, uint8_t state)
{
    if (state)
    {
        self->config.meas_temp = 1;
    }
    else
    {
        self->config.meas_temp = 0;
    }
}

void mpu6050_set_accelerometer_measuring(struct mpu6050 *self, uint8_t state)
{
    if (state)
    {
        self->config.meas_acce = 1;
    }
    else
    {
        self->config.meas_acce = 0;
    }
}

void mpu6050_set_gyroscope_measuring(struct mpu6050 *self, uint8_t state)
{
    if (state)
    {
        self->config.meas_gyro = 1;
    }
    else
    {
        self->config.meas_gyro = 0;
    }
}

void mpu6050_calibrate_gyro(struct mpu6050 *self, uint8_t samples)
{
    self->config.use_calibrate = 1;

    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        read_raw_gyro(self);

        sumX += self->rg.x;
        sumY += self->rg.y;
        sumZ += self->rg.z;

        sigmaX += self->rg.x * self->rg.x;
        sigmaY += self->rg.y * self->rg.y;
        sigmaZ += self->rg.z * self->rg.z;

        sleep_ms(5);
    }

    self->calibration_data.dg.x = sumX / samples;
    self->calibration_data.dg.y = sumY / samples;
    self->calibration_data.dg.z = sumZ / samples;

    self->calibration_data.th.x = sqrtf((sigmaX / samples) - (self->calibration_data.dg.x * self->calibration_data.dg.x));
    self->calibration_data.th.y = sqrtf((sigmaY / samples) - (self->calibration_data.dg.y * self->calibration_data.dg.y));
    self->calibration_data.th.z = sqrtf((sigmaZ / samples) - (self->calibration_data.dg.z * self->calibration_data.dg.z));

    if (self->config.actual_threshold > 0)
    {
        mpu6050_set_threshold(self, 1);
    }
}

void mpu6050_set_threshold(struct mpu6050 *self, uint8_t multiple)
{
    if (multiple > 0)
    {
        if (!self->config.use_calibrate)
        {
            mpu6050_calibrate_gyro(self, 50);
        }

        self->calibration_data.tg.x = self->calibration_data.th.x * multiple;
        self->calibration_data.tg.y = self->calibration_data.th.y * multiple;
        self->calibration_data.tg.z = self->calibration_data.th.z * multiple;
    }
    else
    {
        self->calibration_data.tg.x = 0;
        self->calibration_data.tg.y = 0;
        self->calibration_data.tg.z = 0;
    }

    self->config.actual_threshold = multiple;
}

struct mpu6050_vectorf *mpu6050_get_scaled_accelerometer(struct mpu6050 *self)
{
    if (self->sa.x != 0.0f || self->sa.y != 0.0f || self->sa.z != 0.0f)
    {
        return &self->sa;
    }
    else if (self->ra.x != 0 || self->ra.y != 0 || self->ra.z != 0)
    {
        self->sa.x = self->ra.x * self->config.range_per_digit;
        self->sa.y = self->ra.y * self->config.range_per_digit;
        self->sa.z = self->ra.z * self->config.range_per_digit;

        return &self->sa;
    }

    return NULL;
}

struct mpu6050_vectorf *mpu6050_get_accelerometer(struct mpu6050 *self)
{
    if (self->na.x != 0.0f || self->na.y != 0.0f || self->na.z != 0.0f)
    {
        return &self->na;
    }
    else if (self->ra.x != 0 || self->ra.y != 0 || self->ra.z != 0)
    {
        self->na.x = self->ra.x * self->config.range_per_digit * GRAVITY_CONSTANT;
        self->na.y = self->ra.y * self->config.range_per_digit * GRAVITY_CONSTANT;
        self->na.z = self->ra.z * self->config.range_per_digit * GRAVITY_CONSTANT;

        return &self->na;
    }

    return NULL;
}

struct mpu6050_vectorf *mpu6050_get_gyroscope(struct mpu6050 *self)
{
    if (self->ng.x != 0.0f || self->ng.y != 0.0f || self->ng.z != 0.0f)
    {
        return &self->ng;
    }
    else if (self->ra.x != 0 || self->ra.y != 0 || self->ra.z != 0)
    {
        if (self->config.use_calibrate)
        {
            self->ng.x = (self->rg.x - self->calibration_data.dg.x) * self->config.dps_per_digit;
            self->ng.y = (self->rg.y - self->calibration_data.dg.y) * self->config.dps_per_digit;
            self->ng.z = (self->rg.z - self->calibration_data.dg.z) * self->config.dps_per_digit;
        }
        else
        {
            self->ng.x = self->rg.x * self->config.dps_per_digit;
            self->ng.y = self->rg.y * self->config.dps_per_digit;
            self->ng.z = self->rg.z * self->config.dps_per_digit;
        }

        if (self->config.actual_threshold)
        {
            if (abs(self->ng.x) < self->calibration_data.tg.x)
                self->ng.x = 0.0f;

            if (abs(self->ng.y) < self->calibration_data.tg.y)
                self->ng.y = 0.0f;

            if (abs(self->ng.z) < self->calibration_data.tg.z)
                self->ng.z = 0.0f;
        }

        return &self->ng;
    }

    return NULL;
}

float mpu6050_get_temperature_c(struct mpu6050 *self)
{
    if (self->temperature != 0.0f)
    {
        return self->temperature;
    }
    else if (self->raw_temperature != 0)
    {
        self->temperature = (float)self->raw_temperature / 340 + 36.53f;
        return self->temperature;
    }

    return 0.0f;
}

float mpu6050_get_temperature_f(struct mpu6050 *self)
{
    if (self->temperaturef != 0.0f)
    {
        return self->temperaturef;
    }
    else if (mpu6050_get_temperature_c(self) != 0.0f)
    {
        self->temperaturef = (mpu6050_get_temperature_c(self) * 1.8) + 32;
        return self->temperaturef;
    }

    return 0.0f;
}

void mpu6050_set_gyro_offset_x(struct mpu6050 *self, uint16_t offset)
{
    i2c_write_u16_inline(&self->i2c, GYRO_XOFFS_H, offset);
}

void mpu6050_set_gyro_offset_y(struct mpu6050 *self, uint16_t offset)
{
    i2c_write_u16_inline(&self->i2c, GYRO_YOFFS_H, offset);
}

void mpu6050_set_gyro_offset_z(struct mpu6050 *self, uint16_t offset)
{
    i2c_write_u16_inline(&self->i2c, GYRO_ZOFFS_H, offset);
}

void mpu6050_set_accel_offset_x(struct mpu6050 *self, uint16_t offset)
{
    i2c_write_u16_inline(&self->i2c, ACCEL_XOFFS_H, offset);
}

void mpu6050_set_accel_offset_y(struct mpu6050 *self, uint16_t offset)
{
    i2c_write_u16_inline(&self->i2c, ACCEL_YOFFS_H, offset);
}

void mpu6050_set_accel_offset_z(struct mpu6050 *self, uint16_t offset)
{
    i2c_write_u16_inline(&self->i2c, ACCEL_ZOFFS_H, offset);
}

struct mpu6050_activity *mpu6050_read_activities(struct mpu6050 *self)
{
    return &self->activity;
}

void mpu6050_set_int_motion(struct mpu6050 *self, uint8_t state)
{
    i2c_write_bit_in_reg_inline(&self->i2c, INT_ENABLE, 6, state);
}

void mpu6050_set_int_zero_motion(struct mpu6050 *self, uint8_t state)
{
    i2c_write_bit_in_reg_inline(&self->i2c, INT_ENABLE, 5, state);
}

void mpu6050_set_int_free_fall(struct mpu6050 *self, uint8_t state)
{
    i2c_write_bit_in_reg_inline(&self->i2c, INT_ENABLE, 7, state);
}

void mpu6050_set_dhpf_mode(struct mpu6050 *self, enum MPU6050_DHPF dhpf)
{
    uint8_t value;
    i2c_read_reg(&self->i2c, ACCEL_CONFIG, &value, 1);
    value &= 0xF8;
    value |= dhpf;
    uint8_t data[2] = {ACCEL_CONFIG, value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_dlpf_mode(struct mpu6050 *self, enum MPU6050_DLPF dlpf)
{
    uint8_t value;
    i2c_read_reg(&self->i2c, CONFIG, &value, 1);
    value &= 0xF8;
    value |= dlpf;
    uint8_t data[2] = {CONFIG, value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_motion_detection_threshold(struct mpu6050 *self, uint8_t threshold)
{
    uint8_t data[2] = {MOT_THRESHOLD, threshold};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_motion_detection_duration(struct mpu6050 *self, uint8_t duration)
{
    uint8_t data[2] = {MOT_DURATION, duration};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_zero_motion_detection_threshold(struct mpu6050 *self, uint8_t threshold)
{
    uint8_t data[2] = {ZMOT_THRESHOLD, threshold};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_zero_motion_detection_duration(struct mpu6050 *self, uint8_t duration)
{
    uint8_t data[2] = {ZMOT_DURATION, duration};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_free_fall_detection_threshold(struct mpu6050 *self, uint8_t threshold)
{
    uint8_t data[2] = {FF_THRESHOLD, threshold};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_free_fall_detection_duration(struct mpu6050 *self, uint8_t duration)
{
    uint8_t data[2] = {FF_DURATION, duration};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}