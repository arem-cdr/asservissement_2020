#include "Asservissement_Position.h"

Asservissement_Position::Asservissement_Position()
{
    _codeurD = new Codeur(D5, D6 /*, D0*/);
    _codeurG = new Codeur(D8, D9 /*, D1*/);
    _motors = new Bloc_moteur;
    position_actuel.set_x(0);
    position_actuel.set_y(0);
    angle_actuel = 0;
    nbr_tick_D_prec = 0;
    nbr_tick_G_prec = 0;
}

Asservissement_Position::~Asservissement_Position()
{
    delete _codeurD;
    delete _codeurG;
    delete _motors;
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

    /*------calcul de la trajectoire---------*/

    // determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle
    if (dep_roue_G != dep_roue_D)
    {

        double R = 0;  // rayon du cercle decrit par la trajectoire
        double d = 0;  // vitesse du robot
        double cx = 0; // position du centre du cercle decrit par la trajectoire
        double cy = 0;

        R = ECART_ROUES / 2 * (dep_roue_D + dep_roue_G) / (dep_roue_D - dep_roue_G); // rayon du cercle
        cx = position_actuel.get_x() - R * sin(angle_actuel);
        cy = position_actuel.get_y() + R * cos(angle_actuel);
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

        position_actuel.set_x(cx + R * sin(angle_actuel));
        position_actuel.set_y(cy - R * cos(angle_actuel));
    }
    else if (dep_roue_G == dep_roue_D)
    {                                                                                    // cas où la trajectoire est une parfaite ligne droite
        position_actuel.set_x(position_actuel.get_x() + dep_roue_G * cos(angle_actuel)); //à améliorer
        position_actuel.set_y(position_actuel.get_y() + dep_roue_D * sin(angle_actuel));
    }

    //printf("tick d : %d, tick g : %d, x : %lf, y : %lf. angle : %lf\n", nbr_tick_D, nbr_tick_G, x_actuel, y_actuel, angle*180/PI);
}

double Asservissement_Position::get_angle_deg()
{
    return angle_actuel * 180.0 / PI;
}
