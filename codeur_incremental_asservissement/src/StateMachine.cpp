#include "StateMachine.h"

StateMachine::StateMachine()
{
    _state = NONE;
}

void StateMachine::_update_machine()
{
    State transition = _get_transition();
    if(transition != NONE)
    {
        _exit_state(transition, _state);
        _enter_state(transition, _state);
        _state = transition;
    }
    _state_logic();
}

void StateMachine::_state_logic()
{
    if(_state == LIGNEDROITE)
    {
        
    }
}

State StateMachine::_get_transition()
{
    switch(_state)
    {
        case IDLE:

            break;
        case LIGNEDROITE:

            break;
        case VIRAGE:
            
            break;
        default:
            break;
    }
}

void StateMachine::_enter_state(State new_state, State old_state)
{
    switch(new_state)
    {
        case IDLE:

            break;
        case LIGNEDROITE:

            break;
        case VIRAGE:
            
            break;
        default:
            break;
    }
}

void StateMachine::_exit_state(State new_state, State old_state)
{
    switch(old_state)
    {
        case IDLE:

            break;
        case LIGNEDROITE:

            break;
        case VIRAGE:
            
            break;
        default:
            break;
    }
}