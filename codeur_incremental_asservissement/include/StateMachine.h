#pragma once

#include "mbed.h"
#include "Vector2.h"
#include "Asservissement_Position.h"

enum State {
    NONE,
    IDLE,
    LIGNEDROITE,
    VIRAGE
};

class StateMachine
{
    private:
        State _state;
        double _distance;
        double _angle;
        Asservissement_Position* _parent_asser;
    public:
        StateMachine(Asservissement_Position* parent_asser);
        ~StateMachine();
        void _update_machine();
        void _state_logic();
        State _get_transition();
        void _enter_state(State new_state, State old_state);
        void _exit_state(State new_state, State old_state);
};