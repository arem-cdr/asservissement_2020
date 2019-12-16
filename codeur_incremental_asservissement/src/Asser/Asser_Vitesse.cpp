#include "Asser/Asser_Vitesse.h"

Asser_Vitesse::Asser_Vitesse(double delta_t)
{
    _codeurD = new Codeur(D5, D6);
    _codeurG = new Codeur(D8, D9);
    _motors = new Bloc_moteur;
    _motors->motors_on();

    _delta_t = delta_t;
    _t_last_mesure = 0;
    _t_init_consigne = 0;
    _t_cruise_consigne = 0;

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
    _consigneG_adapte = 0;
    _consigneD_adapte = 0;

    _time.start();

    _PWMG = 0;
    _PWMD = 0;

    _Kpp = 0.03;
    _Kdp = 0.008;
    _Kip = 0.001;

    _accel_max = 0.400;
    _accelG = 0;
    _accelD = 0;
}

Asser_Vitesse::~Asser_Vitesse()
{
    delete _codeurG;
    delete _codeurD;
    delete _motors;
}

void Asser_Vitesse::set_consignes(double consigneG, double consigneD)
{
    _consigneG = consigneG;
    _consigneD = consigneD;

    _init_consignes();
}

void Asser_Vitesse::_mesure_vitesse()
{
    double t = (double) _time.read_ms();
    double ticksG = _codeurG->get_nbr_tick();
    double ticksD = _codeurD->get_nbr_tick();
    _vitesseG = (ticksG - _last_ticksG) * DISTANCE_PAR_TICK_G;
    _vitesseD = (ticksD - _last_ticksD) * DISTANCE_PAR_TICK_D;
    printf("%lf %lf %lf %lf\n", _vitesseG, _vitesseD, _consigneG_adapte * _delta_t, _consigneD_adapte * _delta_t);
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

        _adapter_consignes();

        double errorG = (_consigneG_adapte * _delta_t - _vitesseG);
        double errorD = (_consigneD_adapte * _delta_t - _vitesseD);

        double derivErrorG = (errorG - _last_errorG);
        double derivErrorD = (errorD - _last_errorD);

        _sum_errorG += errorG;
        _sum_errorD += errorD;

        // PID gauche et droit
        _PWMG = _PWMG + _Kpp * errorG + _Kdp * derivErrorG + _Kip * _sum_errorG;
        _PWMD = _PWMD + _Kpp * errorD + _Kdp * derivErrorD + _Kip * _sum_errorD;
        _last_errorG = errorG;
        _last_errorD = errorD;

        //_bridage_moteur(400);
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

void Asser_Vitesse::_init_consignes()
{
    _t_init_consigne = _time.read_ms();
    double t_reaction = 0;
    if(_consigneG < _consigneD)
    {
        t_reaction = (_consigneD - _vitesseD) / _accel_max;
        _accelD = _accel_max;
        _accelG = (_consigneG - _vitesseG) / t_reaction;
    }
    else
    {
        t_reaction = (_consigneG - _vitesseG) / _accel_max;
        _accelG = _accel_max;
        _accelD = (_consigneD - _vitesseD) / t_reaction;
    }
    _t_cruise_consigne = t_reaction;
}

void Asser_Vitesse::_adapter_consignes()
{
    double t_since_init_consigne = _time.read_ms() - _t_init_consigne;

    if(t_since_init_consigne < _t_cruise_consigne)
    {
        _consigneG_adapte = (t_since_init_consigne) * _accelG;
        _consigneD_adapte = (t_since_init_consigne) * _accelD;
    }
    else
    {
        _consigneG_adapte = _consigneG;
        _consigneD_adapte = _consigneD;
    }
    
}