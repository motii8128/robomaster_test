#include "RoboMaster.h"
#include "PID.h"
#include "LoopCycler.h"

using namespace robomaster;

RoboMaster robomas;
// RPM_PID m2006_pid_(3.12, 0.0195, 0.001);
RPM_PID pid(4.43, 0.0062, 0.005);
RPM_PID pid2(4.43, 0.0062, 0.005);

LoopCycler lc(10);

void setup()
{
  Serial.begin(115200);

  robomas.setup();

  robomas.setType(1, MotorType::M3508);
  robomas.setType(2, MotorType::M3508);

  Serial.println("Set up is successfull");
}

void loop()
{
  int out1 = pid.calculation(5000, robomas.getRpm(1), 0.01);
  robomas.setCurrent(1, out1);

  int out2 = pid.calculation(-5000, robomas.getRpm(2), 0.01);
  robomas.setCurrent(2, out2);

  robomas.control();
  robomas.recvFeedBack();
  robomas.recvFeedBack();
  robomas.recvFeedBack();
  robomas.recvFeedBack();

  Serial.print(robomas.getRpm(1));
  Serial.print(',');
  Serial.println(robomas.getRpm(2));

  lc.cycle();
}