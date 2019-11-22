#include "mbed.h"
#include "reglages.h"
#include "Bloc_moteur.h"
#include "L6470.h"

int main()
{
  // Juste un bouton
  DigitalIn button(USER_BUTTON);

  // Instanciation de la classe qui gère les moteurs pas à pas et les codeurs
  Bloc_moteur bloc_moteur(D5, D6, D9, D8);

  // s'assurer que les moteurs ne sont pas bloqués
  bloc_moteur.motors_on();
  // des valeurs de test pour les moteurs (d'après les réglages de reglage.h, -900 <= PWM <= 900)
  bloc_moteur.set_PWM_moteur_D(-200);
  bloc_moteur.set_PWM_moteur_G(200);

  while(1)
  {
    if(button.read() == 0)
    {
      printf("> %d\n", bloc_moteur.get_position_g() % 1200);
    }
  }
}
