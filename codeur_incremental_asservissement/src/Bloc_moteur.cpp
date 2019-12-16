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

void Bloc_moteur::commande_vitesse(float vitesse_G, float vitesse_D)
{ //fonction pour commander les moteurs sans avoir à utiliser set_PWM

    int sens_G = signe(vitesse_G);
    int sens_D = signe(vitesse_D);
    double vitesse_local_G = abs(vitesse_G);
    double vitesse_local_D = abs(vitesse_D);

    if (abs(vitesse_G) > PWM_MAX)
    {
        vitesse_local_G = PWM_MAX;
    }
    if (abs(vitesse_G) < 5)
    {
        vitesse_local_G = 5;
    }
    if (abs(vitesse_D) > PWM_MAX)
    {
        vitesse_local_D = PWM_MAX;
    }
    if (abs(vitesse_D) < 5)
    {
        vitesse_local_D = 5;
    };
    int VG_int = (int)vitesse_local_G * sens_G * COEFF_MOTEUR_G;
    int VD_int = (int)vitesse_local_D * sens_D * COEFF_MOTEUR_D;
    float VG_f = vitesse_local_G * sens_G * COEFF_MOTEUR_G;
    float VD_f = vitesse_local_D * sens_D * COEFF_MOTEUR_D;
    float centieme_D = (VD_f - VD_int) * 100;
    float centieme_G = (VG_f - VG_int) * 100;
    if ((rand() % 100) < centieme_G)
    {
        VG_int += 1;
    }
    if ((rand() % 100) < centieme_D)
    {
        VD_int += 1;
    }
    //printf("vitesseG : %f, vitesseD : %f, %d, %d", VG_f, VD_f, VG_int, VD_int);
    set_PWM_moteur_G(-VG_int/1.0); //le branchements des moteurs est à vérifier ( fonctionne dans l'état actuel du robots
    set_PWM_moteur_D(-VD_int/1.0); //
}