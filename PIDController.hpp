#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

class PIDController
{
    const float kp;
    const float ki;
    const float kd;
    const float clamping_output;
    const uint32_t interval;

    float angle;
    int16_t rpm;
    int16_t amp;
    int8_t temp;
    int16_t target_rpm;

    float integral;
    float previous_error;
    uint32_t count;
public:
    PIDController(float kp, float ki, float kd, float clamping_output, uint32_t interval);
    void set_feedback_values(float angle, int16_t rpm, int16_t amp, uint8_t temp);
    void set_target_rpm(int16_t target_rpm);
    float update();
};

#endif