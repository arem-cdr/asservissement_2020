#pragma once

#include "mbed.h"

enum State {
    NONE,
    LIGNEDROITE,
    TOURNER
};

class StateMachine
{
    private:
        State _state;
    public:
        StateMachine();
        void _state_logic();
        State _get_transition();
        void _enter_state();
        void _exit_state();
};