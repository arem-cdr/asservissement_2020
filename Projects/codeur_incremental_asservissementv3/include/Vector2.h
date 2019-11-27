#pragma once

#include "mbed.h"

// Pour représenter des coordonnées (x, y)
class Vector2
{
    private:
        double x;
        double y;

    public:
        Vector2();
        Vector2(double p_x, double p_y);

        // GETTERS
        double get_x();
        double get_y();

        // SETTERS
        void set_x(double p_x);
        void set_y(double p_y);

        void add_x(double delta_x);
        void add_y(double delta_y);

        // pour obtenir la norme
        double Magnitude();
        // pour normaliser le vecteur
        void Normalize();

        //
        double calculate_angle();

        // surcharge d'operateurs pour l'addition et la multiplication par un double
        Vector2 operator+(Vector2 const &vector2);
        Vector2 operator*(double const &coef);
};
