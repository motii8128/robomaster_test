#ifndef PID_H_
#define PID_H_

namespace robomaster
{
  class RPM_PID
  {
    public:
    RPM_PID(float kp, float ki, float kd);

    float calculation(float target, float actual, float delta_time);

    private:
    float p_gain;
    float i_gain;
    float d_gain;

    float prev_error;
    float integral;
  };
}

#endif