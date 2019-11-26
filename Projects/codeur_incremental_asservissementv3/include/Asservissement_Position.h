#pragma once

#include "mbed.h"
#include "Codeur.h"
#include "Bloc_moteur.h"
#include "Vector2.h"
#include "math_precalc.h"

#define DISTANCE_PAR_TICK_G 8.43
#define DISTANCE_PAR_TICK_D 8.43
#define ECART_ROUES 30000

class Asservissement_Position
{
public:
    Asservissement_Position();
    ~Asservissement_Position();

    Vector2 get_position() const { return position_actuel; }
    void va_au_point(Vector2 position);
    double get_angle_deg();
    double get_angle_rad() const { return angle_actuel; }
    void actualise_position();

    void ligne_droite_basique(double distance);
    void rotation_rel(double angle_vise);
    void rotation_abs(double angle_vise);

private:
    Codeur *_codeurG;
    Codeur *_codeurD;
    Bloc_moteur *_motors;
    Timer time;

    Vector2 position_actuel;
    double angle_actuel;
    long int nbr_tick_D_prec;
    long int nbr_tick_G_prec;
};