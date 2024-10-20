#ifndef ROBOMASTER_H_
#define ROBOMASTER_H_

#include <mcp_can.h>
#include <SPI.h>
#include "PID.h"

extern MCP_CAN can0;

namespace robomaster
{
  enum MotorType
  {
    M2006,
    M3508,
    None,
  };

  class Motor
  {
    public:
    void set_type(MotorType type);
    void set_current(int ampare);

    int get_current();
    int angle_;
    int rpm_;
    MotorType type_;
    int16_t output;
  };

  class RoboMaster
  {
    public:
    void setup();
    void setType(uint8_t id, MotorType type);
    void setCurrent(uint8_t id, int ampare);
    int getRpm(uint8_t id);
    int getAngle(uint8_t id);

    void update();

    private:
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];
    byte txBuf1[8];
    byte txBuf2[8];
    long Pre_millis;
    Motor motor[8];

    bool recv_checker[8];

    bool check_recv(MotorType type)
    {
      if(type != MotorType::None)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
  };
}
#endif