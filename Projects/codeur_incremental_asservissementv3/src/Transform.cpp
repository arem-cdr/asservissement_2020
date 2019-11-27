#include "Transform.h"

Transform::Transform()
{
    _position = new Vector2(0, 0);
    _angle = 0.0;
}

Transform::~Transform()
{
    delete _position;
}

double Transform::get_angle()
{
    return _angle;
}

double Transform::get_x()
{
    return _position->get_x();
}

double Transform::get_y()
{
    return _position->get_y();
}

Vector2 Transform::get_position()
{
    return *_position;
}

void Transform::set_angle(double angle)
{
    _angle = angle;
}

void Transform::set_x(double x)
{
    _position->set_x(x);
}

void Transform::set_y(double y)
{
    _position->set_y(y);
}

void Transform::set_position(Vector2 new_pos)
{
    _position->set_x(new_pos.get_x());
    _position->set_y(new_pos.get_y());
}

void Transform::add_angle(double delta_angle)
{
    _angle += delta_angle;
}

void Transform::add_x(double delta_x)
{
    _position->add_x(delta_x);
}

void Transform::add_y(double delta_y)
{
    _position->add_y(delta_y);
}