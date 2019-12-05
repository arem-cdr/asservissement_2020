#include "Asser/Asser_Vitesse.h"

Asser_Vitesse::Asser_Vitesse(double delta_t)
{
    _codeurD = new Codeur(D5, D6);
    _codeurG = new Codeur(D8, D9);
    _motors = new Bloc_moteur;
    _motors->motors_on();

    _delta_t = delta_t;
    _t_last_mesure = 0;

    _last_ticksG = 0;
    _last_ticksD = 0;

    _vitesseG = 0;
    _vitesseD = 0;

    _sum_errorG = 0;
    _sum_errorD = 0;
    _last_errorG = 0;
    _last_errorD = 0;

    _consigneG = 0;
    _consigneD = 0;

    _time.start();

    _PWMG = 0;
    _PWMD = 0;

    _Kpp = 1;
    _Kdp = 1;
    _Kip = 0;
}

Asser_Vitesse::~Asser_Vitesse()
{
    delete _codeurG;
    delete _codeurD;
    delete _motors;
}

void Asser_Vitesse::set_consigneG(double consigne)
{
    _consigneG = consigne;
}

void Asser_Vitesse::set_consigneD(double consigne)
{
    _consigneD = consigne;
}

void Asser_Vitesse::_mesure_vitesse()
{
    double t = (double) _time.read_ms();
    double ticksG = _codeurG->get_nbr_tick();
    double ticksD = _codeurG->get_nbr_tick();
    _vitesseG = (ticksG - _last_ticksG);
    _vitesseD = (ticksD - _last_ticksD);
    _last_ticksG = ticksG;
    _last_ticksD = ticksD;
    _t_last_mesure = t;
}

void Asser_Vitesse::_update_asser()
{
    double new_t = (double) _time.read_ms();
    if(new_t - _t_last_mesure > _delta_t)
    {
        _mesure_vitesse();

        double errorG = (_consigneG - _vitesseG);
        double errorD = (_consigneD - _vitesseD);

        double derivErrorG = (errorG - _last_errorG);
        double derivErrorD = (errorD - _last_errorD);

        _sum_errorG += errorG;
        _sum_errorD += errorD;

        printf("PWMG:%lf\n", _PWMG);

        // PID gauche et droit
        _PWMG = _PWMG + _Kpp * errorG + _Kdp * derivErrorG + _Kip * _sum_errorG;
        _PWMD = _PWMD + _Kpp * errorD + _Kdp * derivErrorD + _Kip * _sum_errorD;
        _last_errorG = errorG;
        _last_errorD = errorD;

        _bridage_moteur(400);
        _motors->commande_vitesse(_PWMG, _PWMD);

        _t_last_mesure = new_t;
    }
}

void Asser_Vitesse::_bridage_moteur(int vmax)
{
    if (_PWMG > vmax)
    {
        _PWMG = vmax;
    }
    else if (_PWMG < -vmax)
    {
        _PWMG = -vmax;
    }
    if (_PWMD > vmax)
    {
        _PWMD = vmax;
    }
    else if (_PWMD < -vmax)
    {
        _PWMD = -vmax;
    }
}