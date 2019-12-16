#pragma once

#include "mbed.h"
#include "Bloc_moteur.h"
#include "Codeur.h"
#include "math_precalc.h"
#include "reglage.h"

#define DISTANCE_PAR_TICK_G 8.43
#define DISTANCE_PAR_TICK_D 8.43
#define ECART_ROUES 29700

class Asser_Vitesse
{
    private:
        Codeur* _codeurG;
        Codeur* _codeurD;
        Bloc_moteur* _motors;

        Timer _time;
        double _delta_t; // en millis
        double _t_last_mesure; // en millis
        double _t_init_consigne;
        double _t_cruise_consigne;

        double _last_ticksG;
        double _last_ticksD;

        double _vitesseG;
        double _vitesseD;

        double _sum_errorG;
        double _sum_errorD;
        double _last_errorG;
        double _last_errorD;

        double _consigneG;
        double _consigneD;
        double _consigneG_adapte;
        double _consigneD_adapte;

        double _PWMG;
        double _PWMD;

        double _Kpp;
        double _Kip;
        double _Kdp;

        double _accel_max;
        double _accelG;
        double _accelD;

        void _clamp_speed();
        void _bridage_moteur(int vmax);
        void _mesure_vitesse();

        void _init_consignes();
        void _adapter_consignes();
    public:
        Asser_Vitesse(double delta_t);
        ~Asser_Vitesse();

        void set_consignes(double consigneG, double consigneD);
        
        void _update_asser();
};