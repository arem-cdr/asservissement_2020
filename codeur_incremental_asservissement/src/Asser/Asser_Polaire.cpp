#include "Asser/Asser_Polaire.h"
#include <iostream>

using namespace std;

Asser_Polaire::Asser_Polaire()
{
    _codeurG = new Codeur(D5, D6);
    _codeurD = new Codeur(D8, D9);
    _motors = new Bloc_moteur;
    _transform = new Transform;

    time.start();
}

Asser_Polaire::~Asser_Polaire()
{
    delete _codeurG;
    delete _codeurD;
    delete _motors;
}

void Asser_Polaire::actualise_position()
{
    /*
    on suppose les valeurs de vd et vg constantes pendant t, la trajectoire decrite par le robot est alors un cercle
    */

    /*------recuperation de la rotation des roues---------*/
    long int nbr_tick_D = _codeurD->get_nbr_tick();
    long int nbr_tick_G = _codeurG->get_nbr_tick();

    //calcul du nombre de tick
    long int nbr_tick_D_actuel = nbr_tick_D - last_ticksD;
    long int nbr_tick_G_actuel = nbr_tick_G - last_ticksG;

    //sauvegarde
    //nbr_tick_D_prec = nbr_tick_D;
    //nbr_tick_G_prec = nbr_tick_G;

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
    {                                                                            // cas où la trajectoire est une parfaite ligne droite
        _transform->set_x(_transform->get_x() + dep_roue_G * cos(angle_actuel)); //à améliorer
        _transform->set_y(_transform->get_y() + dep_roue_D * sin(angle_actuel));
    }

    _transform->set_angle(angle_actuel);

    //printf("tick d : %d, tick g : %d, x : %lf, y : %lf. angle : %lf\n", nbr_tick_D, nbr_tick_G, x_actuel, y_actuel, angle*180/PI);
}

bool Asser_Polaire::consigne()
{
    if(_check_sampling_time())
    {
        actualise_position();

        double ticksG = _codeurG->get_nbr_tick();
        double ticksD = _codeurD->get_nbr_tick();
        double delta_distG = (ticksG-last_ticksG) * DISTANCE_PAR_TICK_G;
        double delta_distD = (ticksD-last_ticksD) * DISTANCE_PAR_TICK_D;

        last_ticksG = ticksG;
        last_ticksD = ticksD;

        double L = (delta_distG + delta_distD)/2.0;

        double angle = _transform->get_angle();
        dist_parcourue += L;

        _transform->set_angle(angle);

        double error_dist = consigne_dist - dist_parcourue;
        double error_angle = shortest_delta_angle(consigne_angle - angle);

        double accroiss_error_dist = (error_dist - last_error_dist)/_sampling_time;
        double accroiss_error_angle = (error_angle - last_error_angle)/_sampling_time;

        sum_error_dist += error_dist;
        sum_error_angle += error_angle;

        if(abs(error_dist) > ERROR_MIN_DIST || abs(error_angle) > ERROR_MIN_ANGLE)
        {
            double PWMG = 0;
            double PWMD = 0;

            double corr_dist = KppD * error_dist + KipD * sum_error_dist + KdpD * accroiss_error_dist;
            double corr_angle = KppA * error_angle + KipA * sum_error_angle + KdpA * accroiss_error_angle;

            cout << dist_parcourue << endl;
            //cout << corr_dist << ", " << corr_angle << endl;

            PWMG = corr_dist + corr_angle;
            PWMD = corr_dist - corr_angle;
            //_motors->motors_stop_hard_hiz();
            _motors->motors_on();
            _motors->commande_vitesse(PWMG, PWMD);
            return false;
        }
        else
        {
            // si on a atteint la consigne
            _motors->commande_vitesse(0, 0);
            _motors->motors_stop_hard_hiz();
            return true;
        }
    }
    

}

bool Asser_Polaire::_check_sampling_time()
{
    double new_t = time.read_us();
    double delta_t = new_t - _last_t;

    if(delta_t > _sampling_time)
    {
        _last_t = new_t;
    }
    return delta_t > _sampling_time;
}

double Asser_Polaire::shortest_delta_angle(double delta_angle)
{
    if(delta_angle < -PI)
    {
        return delta_angle + 2*PI;
    }
    else if(delta_angle > PI)
    {
        return delta_angle - 2*PI;
    }
    else
    {
        return delta_angle;
    }
    
}