#pragma once

#include "mbed.h"
#include "Codeur.h"
#include "Bloc_moteur.h"
#include "Transform.h"
#include "Vector2.h"
#include "math.h"
#include "reglage.h"

class Asser_Polaire
{
    private:
        Codeur * _codeurG;
        Codeur * _codeurD;
        Bloc_moteur * _motors;

    public:
        Asser_Polaire();
        ~Asser_Polaire();

        void actualise_position();
        bool consigne();
        bool _check_sampling_time();
        double shortest_delta_angle(double delta_angle);

        double consigne_dist = 0;
        double consigne_angle = 0;

        double dist_parcourue = 0;

        double last_ticksG = 0;
        double last_ticksD = 0;

        double KppD = 0.2;
        double KppA = 7500.0;

        double KdpD = .8;
        double KdpA = .8;
        
        double KipD = 0.001;
        double KipA = 0.2;

        double sum_error_dist = 0;
        double sum_error_angle = 0;

        double last_error_dist = 0;
        double last_error_angle = 0;

        Transform * _transform;

        double _sampling_time = 10000.0;
        double _last_t;
        Timer time;
};