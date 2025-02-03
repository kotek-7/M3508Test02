#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

class PIDController
{
    const float kp;
    const float ki;
    const float kd;
    const float clamping_output;
    float integral;
    float previous_error;
    uint32_t count;
public:
    void set_feedback_values(float angle, int16_t rpm, int16_t amp, uint8_t temp);
    void set_target_rpm(int16_t target_rpm);
    float output();
};

#endif PIDCONTROLLER_HPP