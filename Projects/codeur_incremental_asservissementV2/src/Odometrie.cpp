#include "Odometrie.h"

Odometrie::Odometrie()
{
  ecart_roues = ECART_ROUES;

  // Initialisation des codeuses
  codeurG = new Codeur(PIN_CODEUR_G_WHITE, PIN_CODEUR_G_GREEN, D0);
  codeurD = new Codeur(PIN_CODEUR_D_WHITE, PIN_CODEUR_D_GREEN, D1);

  last_g_ticks = 0;
  last_d_ticks = 0;

  position_estimee = new Vector2(0, 0);
  // orientation initiale
  forward_estime = new Vector2(0, 1);
  angle = 0;
}

int Odometrie::get_pos_g()
{
  return codeurG->get_nbr_tick();
}

int Odometrie::get_pos_d()
{
  return codeurD->get_nbr_tick();
}

void Odometrie::update_position()
{
  int g_ticks = codeurG->get_nbr_tick();
  int d_ticks = codeurD->get_nbr_tick();

  int delta_g_ticks = g_ticks - last_g_ticks;
  int delta_d_ticks = d_ticks - last_d_ticks;

  last_g_ticks = g_ticks;
  last_d_ticks = d_ticks;

  float delta_roue_g = delta_g_ticks * PAS_CODEUR;
  float delta_roue_d = delta_d_ticks * PAS_CODEUR;

  if(delta_roue_g * delta_roue_d < 0)//delta_roue_g != delta_roue_d) // si le robot n'avance pas exactement en ligne droite
  {
    double R = 0; // rayon du cercle decrit par la trajectoire
	  double d = 0; // vitesse du robot
    double cx = 0; // position du centre du cercle decrit par la trajectoire
    double cy = 0;

    R = ECART_ROUES / 2 * (delta_roue_d + delta_roue_g) / (delta_roue_d - delta_roue_g); // rayon du cercle
    cx = position_estimee->get_x() - R * sin(angle);
    cy = position_estimee->get_y() + R * cos(angle);
    d = (delta_roue_g + delta_roue_d) / 2;

    // mise à jour des coordonnées du robot
    if (delta_roue_g + delta_roue_d != 0){
        angle += d / R;
    }
    else{
        angle += delta_roue_d * 2 / ECART_ROUES;
    }

    if (angle > 180)
    {
  		angle -= 2*180;
  	}
  	else if (angle <= -180)
    {
  		angle += 2*180;
  	}
    update_forward();

    position_estimee->set_x(cx + R * sin(angle));
    position_estimee->set_y(cy - R * cos(angle));
  }
  else // si la ligne droite est parfaitevoid update_forward()
  {
    *position_estimee =  *position_estimee + (*forward_estime * delta_roue_g);
  }
}

Vector2 Odometrie::get_position()
{
  return *position_estimee;
}

float Odometrie::get_angle()
{
  return angle;
}

void Odometrie::update_forward()
{
  forward_estime->set_x(- sin(angle));
  forward_estime->set_y(cos(angle));
}
