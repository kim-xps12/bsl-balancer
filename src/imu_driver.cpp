#include "imu_driver.h"
#include <cmath>

void ImuDriver::begin(SemaphoreHandle_t i2c_mutex) {
    _i2c_mutex = i2c_mutex;
    _pitch_est = 0.0f;
    _gyro_bias = 0.0f;
}

ImuDriver::Data ImuDriver::update(float dt) {
    Data result = {0, 0, false};

    if (xSemaphoreTake(_i2c_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return result;
    }
    auto mask = M5.Imu.update();
    xSemaphoreGive(_i2c_mutex);

    if (!mask) return result;

    auto imu = M5.Imu.getImuData();

    float ay = imu.accel.y;
    float az = imu.accel.z;
    float gx = imu.gyro.x * DEG_TO_RAD;

    float accel_pitch = atan2f(ay, az);
    float error = accel_pitch - _pitch_est;

    _gyro_bias += KI_MAHONY * error * dt;

    float gyro_corrected = gx + KP_MAHONY * error + _gyro_bias;

    _pitch_est += gyro_corrected * dt;

    result.pitch_deg = _pitch_est * RAD_TO_DEG;
    result.pitch_rate_dps = gyro_corrected * RAD_TO_DEG;
    result.valid = true;
    return result;
}

void ImuDriver::calibrate(uint16_t n_samples, float max_stddev_deg) {
    float sum = 0.0f;
    float sum_sq = 0.0f;

    for (uint16_t i = 0; i < n_samples; i++) {
        if (xSemaphoreTake(_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            M5.Imu.update();
            xSemaphoreGive(_i2c_mutex);
        }
        auto imu = M5.Imu.getImuData();
        float pitch = atan2f(imu.accel.y, imu.accel.z) * RAD_TO_DEG;
        sum += pitch;
        sum_sq += pitch * pitch;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    float mean = sum / n_samples;
    float variance = (sum_sq / n_samples) - (mean * mean);
    float stddev = sqrtf(fabsf(variance));

    if (stddev > max_stddev_deg) {
        Serial.printf("IMU cal: stddev=%.2f > %.2f (vibration?)\n", stddev, max_stddev_deg);
    }

    _pitch_est = mean * DEG_TO_RAD;
    _gyro_bias = 0.0f;

    Serial.printf("IMU cal: pitch=%.2f deg, stddev=%.3f (%d samples)\n", mean, stddev, n_samples);
}
