#pragma once

#include "mbed.h"
#include "Vector2.h"
#include "Asser/Asservissement_Position.h"

class StateMachine
{
    private:
        State _state;
        State _requested_state;
        Asservissement_Position *_parent_asser;
    public:
        StateMachine(State init_state, Asservissement_Position *parent_asser);
        ~StateMachine();
        void _update_machine();
        void _state_logic();
        State _get_transition();
        void _enter_state(State new_state, State old_state);
        void _exit_state(State new_state, State old_state);
        void _request_state(State query);
        State _get_state();
};