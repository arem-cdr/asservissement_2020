#pragma once

#include "mbed.h"
#include "Codeur.h"
#include "Bloc_moteur.h"
#include "Vector2.h"
#include "Transform.h"
#include "math_precalc.h"

#define DISTANCE_PAR_TICK_G 8.43
#define DISTANCE_PAR_TICK_D 8.43
#define ECART_ROUES 30000

class Asservissement_Position
{
public:
    Asservissement_Position();
    ~Asservissement_Position();

    Vector2 get_position() const { return _transform->get_position(); }
    void va_au_point(Vector2 position);
    double get_angle_deg() const { return _transform->get_angle() * 180.0 / PI; }
    double get_angle_rad() const { return _transform->get_angle(); }
    void actualise_position();

    bool ligne_droite_basique(double distance);
    void deplacement_non_bloquant(Vector2 target_pos);
    void rotation_rel(double angle_vise);
    void rotation_abs(double angle_vise);
    void trans_global_to_local();
    void set_premiere_ligne_droite(bool b) { premiere_ligne_droite = b; }

private:
    Codeur *_codeurG;
    Codeur *_codeurD;
    Bloc_moteur *_motors;
    Timer time;

    Transform *_transform;

    bool premiere_ligne_droite = false;

    //repère local
    double angle_rep_local_deg;
    double angle_rep_local;
    double x_local_ini;
    double y_local_ini;
    void create_repere_local();
    // position du robot dans le repere local
    double x_local;
    double y_local;
    void update_position_local();
    //commande vitess
    float vitesse_G;
    float vitesse_D;
    float Kpp = 0.05;
    float Kdp = 10;
    void bridage_moteur(int vmax);
    long int nbr_tick_D_prec;
    long int nbr_tick_G_prec;
};