#include "PIDController.hpp"

PIDController::PIDController(float kp, float ki, float kd, float clamping_output, uint32_t interval)
    : kp(kp), ki(ki), kd(kd), clamping_output(clamping_output), interval(interval)
{
}

void PIDController::set_feedback_values(float angle, int16_t rpm, int16_t amp, uint8_t temp)
{
    this->angle = angle;
    this->rpm = rpm;
    this->amp = amp;
    this->temp = temp;
}

void PIDController::set_target_rpm(int16_t target_rpm)
{
    this->target_rpm = target_rpm;
}

float PIDController::update()
{
    float current_error = static_cast<float>(target_rpm - rpm);
    integral += current_error * static_cast<float>(interval);
    float derivative = (current_error - previous_error) / static_cast<float>(interval);
    float raw_output = kp * current_error + ki * integral + kd * derivative;

    float clamped_output = min(max(raw_output, -clamping_output), clamping_output);
    if (raw_output != clamped_output && (raw_output * current_error > 0))
    {
        integral = 0;
    }

    return clamped_output;
}
