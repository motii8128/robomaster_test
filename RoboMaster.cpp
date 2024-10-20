#include "RoboMaster.h"
#include <Arduino.h>

MCP_CAN can0(10);

int16_t fmap(double x, double in_min, double in_max, int16_t out_min, int16_t out_max) {
  return (x - in_min) * ((double)(out_max - out_min)) / (in_max - in_min) + out_min;
}

namespace robomaster
{
  void Motor::set_type(MotorType type)
  {
    type_ = type;
  }

  void Motor::set_current(int ampare)
  {
    output = ampare;
  }

  int16_t Motor::get_current()
  {
    int16_t max_min = 0;
    if(type_ == MotorType::M2006)
    {
      max_min = 10000;
    }
    else if(type_ == MotorType::M3508)
    {
      max_min = 16384;
    }

    if(output > 0)
    {
      if(output > max_min)
      {
        return max_min;
      }
      else
      {
          return output;
      }
    }
    else if(output < 0)
    {
      if(output < -1*max_min)
      {
        return max_min;
      }
      else
      {
        return output;
      }
    }
    else
    {
      return output;
    }
  }

  void RoboMaster::setup()
  {
    if (can0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    {
      Serial.println("CAN0: Init OK!");
      can0.setMode(MCP_NORMAL);
    }
    else
    {
      Serial.println("CAN0: Init Fail!");
    }

    for(int i = 0; i < 8; i++)
    {
      txBuf1[i] = 0x00;
      txBuf2[i] = 0x00;
      motor[i].set_type(MotorType::None);
      motor[i].angle_ = 0;
      motor[i].rpm_ = 0;
      motor[i].output = 0;

      recv_checker[i] = true;
    }

    len = 0;

    Pre_millis = millis();
  }

  void RoboMaster::setType(uint8_t id, MotorType type)
  {
    motor[id-1].set_type(type);
  }

  void RoboMaster::setCurrent(uint8_t id, int ampare)
  {
    motor[id-1].set_current(ampare);
  }

  int RoboMaster::getRpm(uint8_t id)
  {
    return motor[id-1].rpm_;
  }

  int RoboMaster::getAngle(uint8_t id)
  {
    return motor[id-1].angle_;
  }

  void RoboMaster::update()
  {
    txBuf1[0] = (motor[0].get_current() >> 8) & 0xFF;
    txBuf1[1] = motor[0].get_current() & 0xFF;
    txBuf1[2] = (motor[1].get_current() >> 8) & 0xFF;
    txBuf1[3] = motor[1].get_current() & 0xFF;
    txBuf1[4] = (motor[2].get_current() >> 8) & 0xFF;
    txBuf1[5] = motor[2].get_current() & 0xFF;
    txBuf1[6] = (motor[3].get_current() >> 8) & 0xFF;
    txBuf1[7] = motor[3].get_current() & 0xFF;

    txBuf2[0] = (motor[4].get_current() >> 8) & 0xFF;
    txBuf2[1] = motor[4].get_current() & 0xFF;
    txBuf2[2] = (motor[5].get_current() >> 8) & 0xFF;
    txBuf2[3] = motor[5].get_current() & 0xFF;
    txBuf2[4] = (motor[6].get_current() >> 8) & 0xFF;
    txBuf2[5] = motor[6].get_current() & 0xFF;
    txBuf2[6] = (motor[7].get_current() >> 8) & 0xFF;
    txBuf2[7] = motor[7].get_current() & 0xFF;

    if(millis() - Pre_millis > 50)
    {
      // if(recv_checker[0] && recv_checker[1] && recv_checker[2] && recv_checker[3] && recv_checker[4] && recv_checker[5] && recv_checker[6] && recv_checker[7])
      // {
        can0.sendMsgBuf(0x200, 0, 8, txBuf1);
        can0.sendMsgBuf(0x1FF, 0, 8, txBuf2);
      // }

      Pre_millis = millis();
    }

    // for(int i = 0; i < 8; i++)
    // {
    //   recv_checker[i] = check_recv(motor[i].type_);
    // }

    if (can0.checkReceive() == CAN_MSGAVAIL)
    {
      can0.readMsgBuf(&rxId, &len, rxBuf);

      int16_t angle_data = rxBuf[0] << 8 | rxBuf[1];
      int16_t rpm_data = rxBuf[2] << 8 | rxBuf[3];
      int16_t amp = rxBuf[4] << 8 | rxBuf[5];
      int8_t  temp = rxBuf[6];

      int16_t angle = fmap(angle_data, 0, 8192, 0, 360);

      if(rxId == 0x201)
      {
        motor[0].angle_ = angle;
        motor[0].rpm_ = rpm_data;

        recv_checker[0] = true;
      }
      else if(rxId == 0x202)
      {
        motor[1].angle_ = angle;
        motor[1].rpm_ = rpm_data;

        recv_checker[1] = true;
      }
      else if(rxId == 0x203)
      {
        motor[2].angle_ = angle;
        motor[2].rpm_ = rpm_data;

        recv_checker[2] = true;
      }
      else if(rxId == 0x204)
      {
        motor[3].angle_ = angle;
        motor[3].rpm_ = rpm_data;

        recv_checker[3] = true;
      }
      else if(rxId == 0x205)
      {
        motor[4].angle_ = angle;
        motor[4].rpm_ = rpm_data;

        recv_checker[4] = true;
      }
      else if(rxId == 0x206)
      {
        motor[5].angle_ = angle;
        motor[5].rpm_ = rpm_data;

        recv_checker[5] = true;
      }
      else if(rxId == 0x207)
      {
        motor[6].angle_ = angle;
        motor[6].rpm_ = rpm_data;

        recv_checker[6] = true;
      }
      else if(rxId == 0x208)
      {
        motor[7].angle_ = angle;
        motor[7].rpm_ = rpm_data;

        recv_checker[7] = true;
      }
    }
  }
}

