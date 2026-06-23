#pragma once
#include <M5Unified.h>
#include <freertos/semphr.h>

class ImuDriver {
public:
    struct Data {
        float pitch_deg;
        float pitch_rate_dps;
        bool valid;
    };

    void begin(SemaphoreHandle_t i2c_mutex);
    Data update(float dt);

    void calibrate(uint16_t n_samples = 500, float max_stddev_deg = 0.5f);

    float pitchEstimate() const { return _pitch_est * RAD_TO_DEG; }

private:
    SemaphoreHandle_t _i2c_mutex = nullptr;

    float _pitch_est = 0.0f;
    float _gyro_bias = 0.0f;

    static constexpr float KP_MAHONY = 2.0f;
    static constexpr float KI_MAHONY = 0.005f;
};
