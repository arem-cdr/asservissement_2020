// Code récupéré dans hardware.cpp de l'année dernière et adapté pour être inclut dans une classe
#include "Bloc_moteur.h"

Bloc_moteur::Bloc_moteur()
{
    moteurs_arret = false;
    init_shield = false;

    // Initializing Motor Control Expansion Board.
    dev_spi = new DevSPI(D11, D12, D3);
    x_nucleo_ihm02a1 = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, dev_spi);
    motors = x_nucleo_ihm02a1->get_components();
}

Bloc_moteur::~Bloc_moteur()
{
    delete motors;
    delete dev_spi;
}

void Bloc_moteur::set_PWM_moteur_D(int PWM) // Pour faire tourner les moteurs, ça vient du hello world du shield.
{
    if (!moteurs_arret)
    {
        if (PWM > PWM_MAX)
        {
            motors[0]->prepare_run(StepperMotor::BWD, PWM_MAX); //BWD = backward , FWD = forward , la vitesse doit etre positive
        }
        else if (PWM < -PWM_MAX)
        {
            motors[0]->prepare_run(StepperMotor::FWD, PWM_MAX);
        }
        else if (PWM > 0)
        {
            motors[0]->prepare_run(StepperMotor::BWD, PWM);
        }
        else if (PWM < 0)
        {
            motors[0]->prepare_run(StepperMotor::FWD, -PWM);
        }
        else if (PWM == 0)
        {
            motors[0]->prepare_run(StepperMotor::BWD, 0);
        }
    }
    else
    {
        motors[0]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    }
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void Bloc_moteur::set_PWM_moteur_G(int PWM)
{

    if (!moteurs_arret)
    {
        if (PWM > PWM_MAX)
        {
            motors[1]->prepare_run(StepperMotor::FWD, PWM_MAX);
        }
        else if (PWM < -PWM_MAX)
        {
            motors[1]->prepare_run(StepperMotor::BWD, PWM_MAX);
        }
        else if (PWM > 0)
        {
            motors[1]->prepare_run(StepperMotor::FWD, PWM);
        }
        else if (PWM < 0)
        {
            motors[1]->prepare_run(StepperMotor::BWD, -PWM);
        }
        else if (PWM == 0)
        {
            motors[1]->prepare_run(StepperMotor::BWD, 0);
        }
    }
    else
    {
        motors[1]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    }
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void Bloc_moteur::motors_stop_hard_hiz() //coupe les moteurs et les rends libres.
{
    moteurs_arret = 1;
    motors[0]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    motors[1]->prepare_hard_hiz();
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void Bloc_moteur::motors_stop_low_hiz()
{
    motors[0]->prepare_run(StepperMotor::BWD, 0);
    motors[1]->prepare_run(StepperMotor::BWD, 0);
    motors[0]->perform_prepared_actions();
    motors[1]->perform_prepared_actions();
    moteurs_arret = 1;
}

void Bloc_moteur::motors_on() // il faut activer les moteurs pour qu'il puisse recevoir des commandes PWM.
{
    moteurs_arret = 0;
}
