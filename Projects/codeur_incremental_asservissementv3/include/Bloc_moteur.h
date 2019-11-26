#pragma once

#include "mbed.h"
#include "XNucleoIHM02A1.h"
#include "Codeur.h"
#include "DevSPI.h"
#include "math_precalc.h"
#include "Codeur.h"

#define PWM_MAX 900
#define COEFF_MOTEUR_G 1
#define COEFF_MOTEUR_D 1
class Bloc_moteur
{
private:
  bool moteurs_arret;
  bool init_shield;
  XNucleoIHM02A1 *x_nucleo_ihm02a1; //Création d'une entité pour la carte de contôle des pas à pas
  L6470 **motors;                   //Instance des moteurs
  DevSPI *dev_spi;

  L6470_init_t init[L6470DAISYCHAINSIZE] = {
      /* First Motor.G */
      {
          12,                            /* Motor supply voltage in V. */
          200,                           /* Min number of steps per revolution for the motor. */
          2,                             /* Max motor phase voltage in A. */
          7,                             /* Max motor phase voltage in V. */
          300,                           /* Motor initial speed [step/s]. */
          501,                           /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
          1500.0,                        /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
          992.0,                         /* Motor maximum speed [step/s]. */
          0.0,                           /* Motor minimum speed [step/s]. */
          602.7,                         /* Motor full-step speed threshold [step/s]. */
          5,                             /* Holding kval [V]. */
          5,                             /* Constant speed kval [V]. */
          5,                             /* Acceleration starting kval [V]. */
          5,                             /* Deceleration starting kval [V]. */
          61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
          392.1569e-6,                   /* Start slope [s/step]. */
          643.1372e-6,                   /* Acceleration final slope [s/step]. */
          643.1372e-6,                   /* Deceleration final slope [s/step]. */
          0,                             /* Thermal compensation factor (range [0, 15]). */
          5 * 1000 * 1.10,               /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
          5 * 1000 * 1.00,               /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
          StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
          0xFF,                          /* Alarm conditions enable. */
          0x2E88                         /* Ic configuration. */
      },

      /* Second Motor. */
      {
          12,                            /* Motor supply voltage in V. */
          200,                           /* Min number of steps per revolution for the motor. */
          2,                             /* Max motor phase voltage in A. */
          7,                             /* Max motor phase voltage in V. */
          300,                           /* Motor initial speed [step/s]. */
          501,                           /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
          1500.0,                        /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
          992.0,                         /* Motor maximum speed [step/s]. */
          0.0,                           /* Motor minimum speed [step/s]. */
          602.7,                         /* Motor full-step speed threshold [step/s]. */
          5,                             /* Holding kval [V]. */
          5,                             /* Constant speed kval [V]. */
          5,                             /* Acceleration starting kval [V]. */
          5,                             /* Deceleration starting kval [V]. */
          61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
          392.1569e-6,                   /* Start slope [s/step]. */
          643.1372e-6,                   /* Acceleration final slope [s/step]. */
          643.1372e-6,                   /* Deceleration final slope [s/step]. */
          0,                             /* Thermal compensation factor (range [0, 15]). */
          5 * 1000 * 1.10,               /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
          5 * 1000 * 1.00,               /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
          StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
          0xFF,                          /* Alarm conditions enable. */
          0x2E88                         /* Ic configuration. */
      }};

public:
  // Constructeur
  Bloc_moteur();
  ~Bloc_moteur();
  // Methodes pour définir la tension affectée à chaque moteur
  void set_PWM_moteur_D(int PWM);
  void set_PWM_moteur_G(int PWM);
  void commande_vitesse(float vitesse_G, float vitesse_D);
  // Methode pour bloquer les moteurs
  void motors_stop_hard_hiz();
  void motors_stop_low_hiz();
  // Methode pour autoriser la rotation des moteurs
  void motors_on();
};
