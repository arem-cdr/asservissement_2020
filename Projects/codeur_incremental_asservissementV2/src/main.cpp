#include "mbed.h"
#include "Bloc_moteur.h"
#include "Codeur.h"
#include "L6470.h"
#include "PINS.h"

int main()
{
  DigitalIn button(USER_BUTTON);

  bool go = false;

  // Juste un bouton

  // Instanciation de la classe qui gère les moteurs pas à pas et les codeurs
  Bloc_moteur bloc_moteur;
  Codeur CodeurG(PIN_CODEUR_G_WHITE, PIN_CODEUR_G_GREEN, D0);
  Codeur CodeurD(PIN_CODEUR_D_WHITE, PIN_CODEUR_D_GREEN, D1);

  while (!go)
  {
    bloc_moteur.motors_stop_hard_hiz();
    if (button.read() == 0)
    {
      go = true;
    }
  }
  // s'assurer que les moteurs ne sont pas bloqués
  bloc_moteur.motors_on();
  printf("Init done ! \n");
  // des valeurs de test pour les moteurs (d'après les réglages de reglage.h, -900 <= PWM <= 900)
  bloc_moteur.set_PWM_moteur_D(-200);
  bloc_moteur.set_PWM_moteur_G(200);

  while (1)
  {
    printf("Tick G : %ld |", CodeurG.get_nbr_tick());
    printf("Tick D : %ld\n", CodeurD.get_nbr_tick());
  }
}
