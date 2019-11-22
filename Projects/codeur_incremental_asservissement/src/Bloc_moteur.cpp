// Code récupéré dans hardware.cpp de l'année dernière et adapté pour être inclut dans une classe

#include <mbed.h>
#include "XNucleoIHM02A1.h"
#include "DevSPI.h"
#include "reglages.h"
#include "Codeur.h"
#include "Bloc_moteur.h"

L6470_init_t init[L6470DAISYCHAINSIZE] = {
  /* First Motor. */
  {
      16,                           /* Motor supply voltage in V. */
      200,                           /* Min number of steps per revolution for the motor. */
      7,                           /* Max motor phase voltage in A. */
      15,                          /* Max motor phase voltage in V. */
      200.0,                         /* Motor initial speed [step/s]. */
      200,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
      1000.0,                        /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
      450.0,                         /* Motor maximum speed [step/s]. */
      0.0,                           /* Motor minimum speed [step/s]. */
      602.7,                         /* Motor full-step speed threshold [step/s]. */
      10,                          /* Holding kval [V]. */
      10,                          /* Constant speed kval [V]. */
      10,                          /* Acceleration starting kval [V]. */
      10,                          /* Deceleration starting kval [V]. */
      61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
      392.1569e-6,                   /* Start slope [s/step]. */
      643.1372e-6,                   /* Acceleration final slope [s/step]. */
      643.1372e-6,                   /* Deceleration final slope [s/step]. */
      0,                             /* Thermal compensation factor (range [0, 15]). */
      4.5 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
      4.9 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
      StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
      0xFF,                          /* Alarm conditions enable. */
      0x2E88                         /* Ic configuration. */
  },

  /* Second Motor. */
  {
      16,                           /* Motor supply voltage in V. */
      200,                           /* Min number of steps per revolution for the motor. */
      7,                           /* Max motor phase voltage in A. */
      15,                          /* Max motor phase voltage in V. */
      300.0,                         /* Motor initial speed [step/s]. */
      200,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
      1000.0,                        /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
      450,                         /* Motor maximum speed [step/s]. */
      0.0,                           /* Motor minimum speed [step/s]. */
      602.7,                         /* Motor full-step speed threshold [step/s]. */
      10,                          /* Holding kval [V]. */
      10,                          /* Constant speed kval [V]. */
      10,                          /* Acceleration starting kval [V]. */
      10,                           /* Deceleration starting kval [V]. */
      61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
      392.1569e-6,                   /* Start slope [s/step]. */
      643.1372e-6,                   /* Acceleration final slope [s/step]. */
      643.1372e-6,                   /* Deceleration final slope [s/step]. */
      0,                             /* Thermal compensation factor (range [0, 15]). */
      4.5 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
      4.9 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
      StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
      0xFF,                          /* Alarm conditions enable. */
      0x2E88                         /* Ic configuration. */
  }
};

Bloc_moteur::Bloc_moteur(PinName pin_codeur_g_white, PinName pin_codeur_g_green, PinName pin_codeur_d_white, PinName pin_codeur_d_green)
{
  moteurs_arret = false;
  init_shield = false;

  // Initializing Motor Control Expansion Board.
  x_nucleo_ihm02a1 = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, dev_spi);
  motors = x_nucleo_ihm02a1->get_components();

  dev_spi = new DevSPI(D11, D12, D3);

  // Initialisation des codeuses
  codeurG = new Codeur(pin_codeur_g_white, pin_codeur_g_green);
  codeurD = new Codeur(pin_codeur_d_white, pin_codeur_d_green);
}

void Bloc_moteur::set_PWM_moteur_D(int PWM) // Pour faire tourner les moteurs, ça vient du hello world du shield.
{
    if (!moteurs_arret) {
        if (PWM > PWM_MAX) {
            motors[0]->prepare_run(StepperMotor::BWD, PWM_MAX); //BWD = backward , FWD = forward , la vitesse doit etre positive
        } else if (PWM <-PWM_MAX) {
            motors[0]->prepare_run(StepperMotor::FWD, PWM_MAX);
        } else if (PWM > 0) {
            motors[0]->prepare_run(StepperMotor::BWD, PWM);
        } else if (PWM < 0) {
            motors[0]->prepare_run(StepperMotor::FWD, -PWM);
        } else if (PWM == 0) {
            motors[0]->prepare_run(StepperMotor::BWD, 0);
        }
    } else {
        motors[0]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    }
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void Bloc_moteur::set_PWM_moteur_G(int PWM)
{

    if (!moteurs_arret) {
        if (PWM > PWM_MAX) {
            motors[1]->prepare_run(StepperMotor::FWD, PWM_MAX);
        } else if (PWM <-PWM_MAX) {
            motors[1]->prepare_run(StepperMotor::BWD, PWM_MAX);
        } else if (PWM > 0) {
            motors[1]->prepare_run(StepperMotor::FWD, PWM);
        } else if (PWM < 0) {
            motors[1]->prepare_run(StepperMotor::BWD, -PWM);
        } else if (PWM == 0) {
            motors[1]->prepare_run(StepperMotor::BWD, 0);
        }
    } else {
        motors[1]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    }
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void Bloc_moteur::motors_stop() //coupe les moteurs et les rends libres.
{
    moteurs_arret=1;
    motors[0]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    motors[1]->prepare_hard_hiz();
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void Bloc_moteur::motors_on() // il faut activer les moteurs pour qu'il puisse recevoir des commandes PWM.
{
    moteurs_arret=0;
}

int Bloc_moteur::get_position_g()
{
    return codeurG->get_position();
}

int Bloc_moteur::get_position_d()
{
    return codeurD->get_position();
}

void Bloc_moteur::reset_encoders()
{
  codeurG->set_position(0);
  codeurD->set_position(0);
}
