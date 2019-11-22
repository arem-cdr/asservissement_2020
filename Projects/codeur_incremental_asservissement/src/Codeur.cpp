#include <mbed.h>
#include "Codeur.h"

Codeur::Codeur(PinName white_pin, PinName green_pin)
{
  // routine d'initialisation des interruptions pour le fil blanc et le fil vert

  white = new InterruptIn(white_pin);
  green = new InterruptIn(green_pin);

  white->mode(PullUp);
  green->mode(PullUp);

  // normalement il faut donner un pointeur de fontion en paramètre de rise et fall, mais comme les fonctions qu'on veut
  // appeler font partie de l'instance d'une classe c'est un peu chiant et ça marche pas simplement. Du coup apparament ça sa marche.
  // rise <=> appel sur front montant
  // fall <=> appel sur front descendant
  white->rise(callback(this, &Codeur::rise_white));
  white->fall(callback(this, &Codeur::fall_white));
  green->rise(callback(this, &Codeur::rise_green));
  green->fall(callback(this, &Codeur::fall_green));

  // initialement la roue ne bouge pas, dont les états logiques mémorisés des fils sont à 0
  white_state = 0;
  green_state = 0;

  // la position 0 du capteur est celle dans laquelle il se trouve à son initialisation ici
  position = 0;
}

int Codeur::get_position()
{
  return position;
}
void Codeur::set_position(int p_position)
{
  position = p_position;
}

void Codeur::rise_white()
{
  // le fil blanc passe à 1
  white_state = 1;
}

void Codeur::fall_white()
{
  // le fil blanc passe à 0
  white_state = 0;

  if(green_state == 1)
  {
    position += 1;
  }
  else
  {
    position -= 1;
  }
}

void Codeur::rise_green()
{
  // le fil vert passe à 1
  green_state = 1;
}

void Codeur::fall_green()
{
  // le fil vert passe à 0
  green_state = 0;
  
  if(white_state == 1)
  {
    position -= 1;
  }
  else
  {
    position += 1;
  }
}
