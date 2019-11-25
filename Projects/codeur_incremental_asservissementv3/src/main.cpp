#include "mbed.h"
#include "Bloc_moteur.h"
#include "Codeur.h"
#include "Asservissement_Position.h"
#include "Vector2.h"

int main()
{
  RawSerial pc(USBTX, USBRX);

  pc.printf("Start ! \n");
  DigitalIn button(USER_BUTTON);
  Vector2 pos;
  double angle;
  bool go = false;

  Asservissement_Position Asser;
  while (!go)
  {
    if (button.read() == 0)
    {
      go = true;
    }
  }
  pc.printf("Init done ! \n");

  while (1)
  {
    Asser.actualise_position();
    pos = Asser.get_position();
    angle = Asser.get_angle_deg();
    pc.printf("x : %lf, y : %lf, a : %lf\n", pos.get_x(), pos.get_y(), angle);
  }
}
