#include "Asser/Asservissement_Position.h"

Asservissement_Position::Asservissement_Position()
{
    _codeurD = new Codeur(D5, D6 /*, D0*/);
    _codeurG = new Codeur(D8, D9 /*, D1*/);
    _motors = new Bloc_moteur;

    nbr_tick_D_prec = 0;
    nbr_tick_G_prec = 0;

    _transform = new Transform;

    _info_consigne = new Transform;
    _consigne = State::NONE;

    time.start();
}

Asservissement_Position::~Asservissement_Position()
{
    delete _codeurD;
    delete _codeurG;
    delete _motors;
    delete _transform;
    delete _info_consigne;
}

void Asservissement_Position::_actualise_position()
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
    {                                                                            // cas où la trajectoire est une parfaite ligne droite
        _transform->set_x(_transform->get_x() + dep_roue_G * cos(angle_actuel)); //à améliorer
        _transform->set_y(_transform->get_y() + dep_roue_D * sin(angle_actuel));
    }

    _transform->set_angle(angle_actuel);

    //printf("tick d : %d, tick g : %d, x : %lf, y : %lf. angle : %lf\n", nbr_tick_D, nbr_tick_G, x_actuel, y_actuel, angle*180/PI);
}

/*void Asservissement_Position::_actualise_position()
{
    // (Hugo:) VERSION PERSO POUR COMPRENDRE CELLE QU'ON UTILISE, PAS TERMINEE, PASSEZ VOTRE CHEMIN
    if(_check_sampling_time())
    {
        long int ticksG = _codeurG->get_nbr_tick();
        long int ticksD = _codeurD->get_nbr_tick();

        //calcul du nombre de tick
        long int deltaTicksG = ticksG - nbr_tick_G_prec;
        long int deltaTicksD = ticksD - nbr_tick_D_prec;

        //sauvegarde
        nbr_tick_G_prec = ticksG;
        nbr_tick_D_prec = ticksD;

        double deltaDistG = deltaTicksG * DISTANCE_PAR_TICK_G; // deplacement des roues
        double deltaDistD = deltaTicksD * DISTANCE_PAR_TICK_D;

        double angle_actuel = _transform->get_angle();

        if(deltaDistG != deltaDistD)
        {
            double R = (ECART_ROUES/2.0) * (deltaDistG + deltaDistD)/(abs(deltaDistD - deltaDistG));
        }
        else // if(deltaDistG == deltaDistD)
        {

        }
    }
}*/

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
        _actualise_position();
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
    wait_us(10000);
    _motors->motors_stop_hard_hiz();
}

bool Asservissement_Position::ligne_droite_basique(double distance)
{
    _motors->motors_on();
    if (premiere_ligne_droite)
    {
        _actualise_position();
        create_repere_local();
        update_position_local();
        premiere_ligne_droite = false;
    }

    if (distance - x_local > 0)
    {
        _actualise_position();
        update_position_local();
        vitesse_G = (distance - x_local) / 70;
        vitesse_D = vitesse_G;
        bridage_moteur(400);

        double angle_actuel = get_angle_deg();
        vitesse_G = vitesse_G + Kpp * y_local + Kdp * diff_angle(angle_rep_local_deg, angle_actuel);
        vitesse_D = vitesse_D - Kpp * y_local - Kdp * diff_angle(angle_rep_local_deg, angle_actuel);
        _motors->commande_vitesse(vitesse_G, vitesse_D);
        return false;
    }
    else
    {
        rotation_abs(angle_rep_local_deg);
        return true;
    }
}

void Asservissement_Position::rotation_abs(double angle_vise)
{
    _actualise_position();
    double angle_rel = borne_angle_d(angle_vise - get_angle_deg());
    rotation_rel(angle_rel);
}

void Asservissement_Position::create_repere_local()
{
    double x_ini = _transform->get_x();
    double y_ini = _transform->get_y();
    angle_rep_local_deg = _transform->get_angle();
    angle_rep_local = angle_rep_local_deg * PI / 180.0;
    x_local_ini = x_ini * cos(angle_rep_local) + y_ini * sin(angle_rep_local);
    y_local_ini = y_ini * cos(angle_rep_local) - x_ini * sin(angle_rep_local);
}

void Asservissement_Position::update_position_local()
{
    double x_actuel = _transform->get_x();
    double y_actuel = _transform->get_y();
    x_local = x_actuel * cos(angle_rep_local) + y_actuel * sin(angle_rep_local) - x_local_ini;
    y_local = y_actuel * cos(angle_rep_local) - x_actuel * sin(angle_rep_local) - y_local_ini;
}

void Asservissement_Position::bridage_moteur(int vmax)
{
    if (vitesse_G > vmax)
    {
        vitesse_G = vmax;
        vitesse_D = vmax;
    }
    if (vitesse_G < -vmax)
    {
        vitesse_G = -vmax;
        vitesse_D = -vmax;
    }
}

void Asservissement_Position::idle()
{
    if(_check_sampling_time())
    {
        _actualise_position();
    }
    else
    {
        
    }
}

bool Asservissement_Position::go_to_target()
{
    if(_check_sampling_time())
    {
        _actualise_position();

        Vector2 consigne_locale(_info_consigne->get_x() - _transform->get_x(), _info_consigne->get_y() - _transform->get_y());
        double delta_angle_consigne = _shortest_delta_angle(consigne_locale.calculate_angle() - _transform->get_angle());
        double distance_to_target = consigne_locale.Magnitude();    
        //printf("state:%d\na:%lf\ndelta:%lf\ndist:%lf\n\n", _consigne, _transform->get_angle()*180/PI, delta_angle_consigne*180/PI, distance_to_target);
        printf("%lf %lf %lf %lf %lf %lf delta:%lf\n", _transform->get_x(), _transform->get_y(), _transform->get_angle()*180/PI, _info_consigne->get_x(), _info_consigne->get_y(), _info_consigne->get_angle()*180/PI, delta_angle_consigne);


        if(distance_to_target > 500.0)
        {
            double signG = 1.0;
            double signD = 1.0;
            if(delta_angle_consigne < -PI/4)
            {
                signD = -1.0;
            }
            else if(delta_angle_consigne > PI/4)
            {
                signG = -1.0;
            }

            vitesse_G = distance_to_target * 0.006f;
            vitesse_D = distance_to_target * 0.006f;
            bridage_moteur(300);

            double target_local_x = (consigne_locale.get_x() * cos(delta_angle_consigne) + consigne_locale.get_y() * sin(delta_angle_consigne));
            double target_local_y = (consigne_locale.get_y() * cos(delta_angle_consigne) - consigne_locale.get_x() * sin(delta_angle_consigne));

            vitesse_G += + Kpp * target_local_y - Kdp * delta_angle_consigne;
            vitesse_D += - Kpp * target_local_y + Kdp * delta_angle_consigne;

            vitesse_G *= signG;
            vitesse_D *= signD;
            bridage_moteur(300);
            //_motors->motors_on();
            _motors->motors_stop_hard_hiz();
            _motors->commande_vitesse(vitesse_G, vitesse_D);

            return false;
        }
        else
        {
            _motors->commande_vitesse(0, 0);
            _motors->motors_stop_hard_hiz();
            _consigne = State::IDLE;
            return true;
        }
    }
    else
    {

    }
    
    
}

bool _straight_line(double distance)
{
    
}

bool Asservissement_Position::turn_to_abs_angle()
{
    if(_check_sampling_time())
    {
        _actualise_position();

        double delta_angle_consigne = _shortest_delta_angle(_info_consigne->get_angle() - _transform->get_angle());
        //printf("state:%d\na:%lf\ndelta:%lf\n\n", _consigne, _transform->get_angle()*180/PI, delta_angle_consigne*180/PI);
        printf("%lf %lf %lf %lf %lf %lf\n", _transform->get_x(), _transform->get_y(), _transform->get_angle()*180/PI, _info_consigne->get_x(), _info_consigne->get_y(), _info_consigne->get_angle()*180/PI);

        if(abs(delta_angle_consigne) > PI/48.0)
        {
            vitesse_G = - (delta_angle_consigne) * Kdp/2.0;
            vitesse_D = + (delta_angle_consigne) * Kdp/2.0;
            bridage_moteur(300);
            //_motors->motors_on();
            _motors->motors_stop_hard_hiz();
            _motors->commande_vitesse(vitesse_G, vitesse_D);

            return false;
        }
        else
        {
            _motors->commande_vitesse(0, 0);
            _motors->motors_stop_hard_hiz();
            _consigne = State::IDLE;
            return true;
        }
    }
    else
    {
    }
}

State Asservissement_Position::get_consigne()
{
    return _consigne;
}

Transform Asservissement_Position::get_info_consigne()
{
    return *_info_consigne;
}

Vector2 Asservissement_Position::get_delta_to_consigne()
{
    Vector2 delta_to_consigne(_info_consigne->get_x() - _transform->get_x(), _info_consigne->get_y() - _transform->get_y());
    return delta_to_consigne;
}

double Asservissement_Position::_shortest_delta_angle(double delta_angle)
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

bool Asservissement_Position::_check_sampling_time()
{
    double new_t = time.read_us();
    double delta_t = new_t - _last_t;

    if(delta_t > _sampling_time)
    {
        _last_t = new_t;
    }
    return delta_t > _sampling_time;
}

void Asservissement_Position::set_consigne(State consigne)
{
    _consigne = consigne;
}

void Asservissement_Position::set_info_consigne(double x, double y, double angle)
{
    _info_consigne->set_x(x);
    _info_consigne->set_y(y);
    _info_consigne->set_angle(angle);
}