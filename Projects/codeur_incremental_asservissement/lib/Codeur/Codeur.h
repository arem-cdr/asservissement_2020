#ifndef CODEUR_HPP
#define CODEUR_HPP

#include <mbed.h>

// Un objet de type Codeuse représente un codeur incremental
class Codeur
{
  private:
  // Les deux interrupts ensuite initialisées pour mettre à jour la valeur du compteur sur front montant et descendant
  /*
  * Les deux fils du codeur produisent une tension créneau déphasée.
  * Le fil blanc produit des créneaux avec une légère avance sur le vert lorsque le codeur tourne dans un sens,
  * et le contraire dans l'autre sens.
  * Par exemple: sur un front montant pour la tension du fil blanc, si la tension du fil vert est 0V, on peut en déduire que c'est le signal
  * créneau du fil blanc qui est en avance: donc on a le sens dans lequel tourne la roue.
  * En répétant ce procédé pour chaque front montant et descendant, on tient à jour un compteur de la position absolue de la roue initialisée
  * dans le constructeur de la classe Bloc_moteur.
  */
  InterruptIn* white;
  InterruptIn* green;

  // mémoire de l'état logique des fils blanc et vert (car on fonctionne de façon synchrone sur front montant et descendant)
  int white_state;
  int green_state;
  // la position angulaire de la roue
  int position;

  public:
  // Constructeur
  Codeur(PinName white_pin, PinName green_pin); // pour le test : white D5 et green D6
  // Getter et Setter de position (de cette façon, si on veut changer la façon d'obtenir la positon, le code qui utilise cet objet ne devient pas obsolète)
  int get_position();
  void set_position(int p_position);

  private:
  // Les méthodes appelées par les interruptions
  void rise_white();
  void fall_white();
  void rise_green();
  void fall_green();
};

#endif
