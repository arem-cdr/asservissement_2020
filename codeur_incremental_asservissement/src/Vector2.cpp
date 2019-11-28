#include "Vector2.h"
#include "math_precalc.h"

Vector2::Vector2()
{
    x = 0;
    y = 0;
}

Vector2::Vector2(double p_x, double p_y)
{
    x = p_x;
    y = p_y;
}

double Vector2::get_x()
{
    return x;
}
void Vector2::set_x(double p_x)
{
    x = p_x;
}
double Vector2::get_y()
{
    return y;
}
void Vector2::set_y(double p_y)
{
    y = p_y;
}

void Vector2::add_x(double delta_x)
{
    x += delta_x;
}

void Vector2::add_y(double delta_y)
{
    y += delta_y;
}

double Vector2::Magnitude()
{
    return pow((pow(x, 2.0f) + pow(y, 2.0f)), 0.5f);
}

void Vector2::Normalize()
{
    double norm = this->Magnitude();
    x = x / norm;
    y = y / norm;
}

double Vector2::calculate_angle()
{
    if (x >= 0)
    {
        return atan(y / x);
    }
    else
    {
        return PI - atan(-y / x);
    }
}

Vector2 Vector2::operator+(Vector2 const &vector2)
{
    Vector2 new_vect(x + vector2.x, y + vector2.y);
    return new_vect;
}

Vector2 Vector2::operator*(double const &coef)
{
    Vector2 new_vect(x, y);
    new_vect.x *= coef;
    new_vect.y *= coef;
    return new_vect;
}