#pragma once

#include "Vector2.h"

class Transform
{
    private:
        Vector2* _position;
        double _angle;

    public:
        Transform();
        ~Transform();
        
        // GETTERS
        double get_angle();
        double get_x();
        double get_y();
        Vector2 get_position();

        // SETTERS
        void set_angle(double angle);
        void set_x(double x);
        void set_y(double y);
        void set_position(Vector2 new_pos);

        void add_angle(double delta_angle);
        void add_x(double delta_x);
        void add_y(double delta_y);
};