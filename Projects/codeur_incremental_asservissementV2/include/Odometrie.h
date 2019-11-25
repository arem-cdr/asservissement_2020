#ifndef ODOMETRIE_H
#define ODOMETRIE_H

#include "mbed.h"
#include "Codeur.h"
#include "Vector2.h"
#include "PINS.h"
#include "Codeur.h"

//propre a chaque robot
#define ECART_ROUES 300
#define RAYON_CODEUR 32.0f

#define PAS_CODEUR 2.0f * M_PI * RAYON_CODEUR / 1200.0f

#define DISTANCE_PAR_TICK_D 8.43 // si le robot va trop loin, à augmenter
#define DISTANCE_PAR_TICK_G 8.43

class Odometrie
{
private:
  //parametres de dimension du robot necessaires pour tenir la position à jour
  float ecart_roues;
  // Declaration des codeurs
  Codeur* codeurG;
  Codeur* codeurD;
  int last_g_ticks;
  int last_d_ticks;
  // la position estimée par rapport à la position d'initialisation du robot
  Vector2* position_estimee;
  // la direction estimee dans vers laquelle le robot pointe
  Vector2* forward_estime;
  float angle;
public:
  Odometrie();
  int get_pos_g();
  int get_pos_d();
  void update_position();
  Vector2 get_position();
  float get_angle();
  void update_forward();
};

#endif
