#include "mbed.h"
#include "Bloc_moteur.h"
#include "Codeur.h"
#include "Asser/Asservissement_Position.h"
#include "Asser/Asser_Vitesse.h"
#include "Vector2.h"
#include "StateMachine.h"

int main()
{
  RawSerial pc(USBTX, USBRX);
  pc.printf("Start ! \n");
  DigitalIn button(USER_BUTTON);
  Vector2 pos;
  bool go = false;

  Asservissement_Position asser;
  //Asser_Vitesse asserV(10);

  while (!go)
  {
    if (button.read() == 0)
    {
      go = true;
    }
  }
  /*pc.printf("Init done ! \n");
  bool b = false;
  while (!b)
  {
    b = asser.ligne_droite_basique(30000);
    printf("b : %d\n", b);
  }*/
  /*while (1)
  {
    asser.actualise_position();
    pos = asser.get_position();
    angle = asser.get_angle_deg();
    pc.printf("x : %lf, y : %lf, a : %lf\n", pos.get_x(), pos.get_y(), angle);
  }*/

  // CODE POUR LE TEST DE LA STATEMACHINE
  StateMachine sm(State::IDLE, &asser);
  double consignes[5][3] = {{50000.0, .0, .0}, {50000.0, 00.0, PI/2.0}, {50000.0, 10000.0, .0}, {.0, .0, -PI/2}, {50000.0, -10000.0, .0}};
  State tete_consignes[5] = {State::LIGNEDROITE, State::VIRAGE, State::LIGNEDROITE, State::VIRAGE, State::LIGNEDROITE};
  int curseur_consignes = 0;
  while(1)
  {
    wait_us(200000);
    if(sm._get_state() == State::IDLE && curseur_consignes < 5)// && button.read() == 0)
    {
      asser.set_info_consigne(consignes[curseur_consignes][0], consignes[curseur_consignes][1], consignes[curseur_consignes][2]);
      sm._request_state(tete_consignes[curseur_consignes]);
      curseur_consignes++;
    }
    asser.actualise_position(); // a peut-etre mettre dans la SM
    sm._update_machine();
    Vector2 delta_to_target;
    delta_to_target = asser.get_delta_to_consigne();
    //printf(">>%lf\n>>%lf\n\n", delta_to_target.get_x(), delta_to_target.get_y());
  }
  /*asserV.set_consigneG(20);
  asserV.set_consigneD(20);
  while(1)
  {
    asserV._update_asser();
  }*/
  printf("DONE !!!!!!!!\n");
}
