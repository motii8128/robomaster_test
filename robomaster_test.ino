#include "RoboMaster.h"

using namespace robomaster;

RoboMaster robomas;
RPM_PID pid1(1.8, 0.1, 0.03);

void setup()
{
  Serial.begin(115200);

  robomas.setup();

  robomas.setType(1, MotorType::M2006);
}

void loop()
{
  auto out1 = pid1.calculation(-4000, robomas.getRpm(1), 0.02);
  robomas.setCurrent(1, out1);

  robomas.update();
  Serial.println(robomas.getRpm(1));
}