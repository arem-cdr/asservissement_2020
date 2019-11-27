#include "Asservissement_Position.h"

Asservissement_Position::Asservissement_Position()
{
    _codeurD = new Codeur(D5, D6 /*, D0*/);
    _codeurG = new Codeur(D8, D9 /*, D1*/);
    _motors = new Bloc_moteur;

    _transform = new Transform();

    nbr_tick_D_prec = 0;
    nbr_tick_G_prec = 0;
}

Asservissement_Position::~Asservissement_Position()
{
    delete _codeurD;
    delete _codeurG;
    delete _motors;
    delete _transform;
}

void Asservissement_Position::actualise_position()
{
    /*
    on suppose les valeurs de vd et vg constantes pendant t, la trajectoire decrite par le robot est alors un cercle
    */

    /*------recuperation de la rotation des roues---------*/
    long int nbr_tick_D = _codeurD->get_nbr_tick();
    long int nbr_tick_G = _codeurG->get_nbr_tick();

    //calcul du nombre de tick
    long int nbr_tick_D_actuel = nbr_tick_D - nbr_tick_D_prec;
    long int nbr_tick_G_actuel = nbr_tick_G - nbr_tick_G_prec;

    //sauvegarde
    nbr_tick_D_prec = nbr_tick_D;
    nbr_tick_G_prec = nbr_tick_G;

    double dep_roue_G = nbr_tick_G_actuel * DISTANCE_PAR_TICK_G; // deplacement des roues
    double dep_roue_D = nbr_tick_D_actuel * DISTANCE_PAR_TICK_D;

    double angle_actuel = _transform->get_angle();

    /*------calcul de la trajectoire---------*/

    // determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle
    if (dep_roue_G != dep_roue_D)
    {

        double R = 0;  // rayon du cercle decrit par la trajectoire
        double d = 0;  // vitesse du robot
        double cx = 0; // position du centre du cercle decrit par la trajectoire
        double cy = 0;

        R = ECART_ROUES / 2 * (dep_roue_D + dep_roue_G) / (dep_roue_D - dep_roue_G); // rayon du cercle
        cx = _transform->get_x() - R * sin(angle_actuel);
        cy = _transform->get_y() + R * cos(angle_actuel);
        d = (dep_roue_G + dep_roue_D) / 2;

        // mise à jour des coordonnées du robot
        if (dep_roue_G + dep_roue_D != 0)
        {
            angle_actuel += d / R;
        }
        else
        {
            angle_actuel += dep_roue_D * 2 / ECART_ROUES;
        }

        angle_actuel = borne_angle_r(angle_actuel);

        _transform->set_x(cx + R * sin(angle_actuel));
        _transform->set_y(cy - R * cos(angle_actuel));
    }
    else if (dep_roue_G == dep_roue_D)
    {                                                                                    // cas où la trajectoire est une parfaite ligne droite
        _transform->set_x(_transform->get_x() + dep_roue_G * cos(angle_actuel)); //à améliorer
        _transform->set_y(_transform->get_y() + dep_roue_D * sin(angle_actuel));
    }

    _transform->set_angle(angle_actuel);

    //printf("tick d : %d, tick g : %d, x : %lf, y : %lf. angle : %lf\n", nbr_tick_D, nbr_tick_G, x_actuel, y_actuel, angle*180/PI);
}

void Asservissement_Position::rotation_rel(double angle_vise)
{
    // rotation de angle_vise
    _motors->motors_on();
    double vitesse = 180;
    int sens;
    angle_vise += get_angle_deg();
    borne_angle_d(angle_vise);
    if (diff_angle(get_angle_deg(), angle_vise) <= 0)
    {
        sens = -1;
    }
    else
    {
        sens = 1;
    }
    while ((sens * diff_angle(get_angle_deg(), angle_vise) > 0) || abs(diff_angle(get_angle_deg(), angle_vise)) > 100)
    {
        actualise_position();
        vitesse = 3 * sens * abs(diff_angle(get_angle_deg(), angle_vise));
        if (vitesse > 90)
        {
            vitesse = 90;
        }
        if (vitesse < -90)
        {
            vitesse = -90;
        }
        _motors->commande_vitesse(-vitesse, vitesse);
    }

    _motors->commande_vitesse(0, 0);
    wait(0.1);
    _motors->motors_stop_hard_hiz();
}

void Asservissement_Position::ligne_droite_basique(double distance)
{
    // le robot avance en ligne droite sur une distance donnée, à la vitesse voulue (entre 0 et 900)
    _motors->motors_on();
    actualise_position();
    double x_ini = _transform->get_x();
    double y_ini = _transform->get_y();
    double angle_vise_deg = _transform->get_angle();
    double angle_vise = angle_vise_deg * PI / 180.0;

    double x_local_ini = x_ini * cos(angle_vise) + y_ini * sin(angle_vise);
    double y_local_ini = y_ini * cos(angle_vise) - x_ini * sin(angle_vise);

    double x_actuel = _transform->get_x();
    double y_actuel = _transform->get_y();

    double x_local = x_actuel * cos(angle_vise) + y_actuel * sin(angle_vise) - x_local_ini;
    double y_local = y_actuel * cos(angle_vise) - x_actuel * sin(angle_vise) - y_local_ini;

    //long int y_local_prec = y_local;
    float vitesse_G;
    float vitesse_D;

    float Kpp = 0.05;
    float Kdp = 10;
    while (distance - x_local > 0)
    {
        vitesse_G = (distance - x_local) / 70;
        vitesse_D = vitesse_G;
        if (vitesse_G > 400)
        {
            vitesse_G = 400;
            vitesse_D = 400;
        }
        if (vitesse_G < -400)
        {
            vitesse_G = -400;
            vitesse_D = -400;
        }
        double angle_actuel = get_angle_deg();
        vitesse_G = vitesse_G + Kpp * y_local + Kdp * diff_angle(angle_vise_deg, angle_actuel);
        vitesse_D = vitesse_D - Kpp * y_local - Kdp * diff_angle(angle_vise_deg, angle_actuel);

        actualise_position();
        x_actuel = _transform->get_x();
        y_actuel = _transform->get_y();
        _motors->commande_vitesse(vitesse_G, vitesse_D);
        x_local = x_actuel * cos(angle_vise) + y_actuel * sin(angle_vise) - x_local_ini;
        y_local = y_actuel * cos(angle_vise) - x_actuel * sin(angle_vise) - y_local_ini;
    }
    rotation_abs(angle_vise_deg);
}

void Asservissement_Position::rotation_abs(double angle_vise)
{
    actualise_position();
    double angle_rel = borne_angle_d(angle_vise - get_angle_deg());
    rotation_rel(angle_rel);
}

void Asservissement_Position::deplacement_non_bloquant(Vector2 target_pos)
{
    _motors->motors_on();
    actualise_position();

    /*double local_x = position_actuel.get_x() * cos()
    double distance =*/
}