#include "PID.h"

namespace robomaster
{
  RPM_PID::RPM_PID(float kp, float ki, float kd) : p_gain(kp), i_gain(ki), d_gain(kd)
  {
    integral = 0.0;
    prev_error = 0.0;
  }

  float RPM_PID::calculation(float target, float actual, float delta_time)
  {
    float error = target - actual;
    integral += error * delta_time;

    float derivative = (error - prev_error) / delta_time;
    prev_error = error;

    return (p_gain * error + i_gain * integral + d_gain * derivative)*0.29411764705882354;
  }
}