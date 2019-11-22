#include <mbed.h>
#include "XNucleoIHM02A1.h"
#include "Codeur.h"

class Bloc_moteur
{
private:
  bool moteurs_arret;
  bool init_shield;
  XNucleoIHM02A1 *x_nucleo_ihm02a1; //Création d'une entité pour la carte de contôle des pas à pas
  L6470 **motors; //Instance des moteurs
  DevSPI* dev_spi;
  // Declaration des codeurs
  Codeur* codeurG;
  Codeur* codeurD;
  //Serial pc(USBTX, USBRX); // tx, rx //la liaison serie pc-robot fut redéfini autre part.
public:
  // Constructeur
  Bloc_moteur(PinName pin_codeuse_g_white, PinName pin_codeuse_g_green, PinName pin_codeuse_d_white, PinName pin_codeuse_d_green);
  // Methodes pour définir la tension affectée à chaque moteur
  void set_PWM_moteur_D(int PWM);
  void set_PWM_moteur_G(int PWM);
  // Methode pour bloquer les moteurs
  void motors_stop();
  // Methode pour autoriser la rotation des moteurs
  void motors_on();
  // Methodes pour obtenir la position angulaire des codeurs (pour l'instant entre -1200 et 1200)
  int get_position_g();
  int get_position_d();
  // Methode pour mettre la position des deux codeurs à zéro
  void reset_encoders();
};
