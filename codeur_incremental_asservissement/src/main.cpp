#include "mbed.h"
#include "Bloc_moteur.h"
#include "Codeur.h"
#include "Asservissement_Position.h"
#include "Vector2.h"

int main()
{
  RawSerial pc(USBTX, USBRX);
  int i = 0;
  pc.printf("Start ! \n");
  DigitalIn button(USER_BUTTON);
  Vector2 pos;
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
  bool b = false;
  while (!b)
  {
    b = Asser.ligne_droite_basique(30000);
    printf("b : %d\n", b);
  }
  /*while (1)
  {
    Asser.actualise_position();
    pos = Asser.get_position();
    angle = Asser.get_angle_deg();
    pc.printf("x : %lf, y : %lf, a : %lf\n", pos.get_x(), pos.get_y(), angle);
  }*/
}
