#include "StateMachine.h"
#include "Bloc_moteur.h"

StateMachine::StateMachine(State init_state, Asservissement_Position* parent_asser)
{
    _state = init_state;
    _parent_asser = parent_asser;
}

StateMachine::~StateMachine()
{

}

void StateMachine::_update_machine()
{
    //printf("state:%d\n", _state);
    State transition = _get_transition();
    if(transition != State::NONE)
    {
        _exit_state(transition, _state);
        _enter_state(transition, _state);
        _state = transition;
    }
    _state_logic();
}

void StateMachine::_state_logic()
{
    if(_state == State::LIGNEDROITE)
    {
        _parent_asser->go_to_target();
    }
    else if(_state == State::VIRAGE)
    {
        _parent_asser->turn_to_abs_angle();
    }
    else
    {
        _parent_asser->idle();
    }
    
}

State StateMachine::_get_transition()
{
    switch(_state)
    {
        case State::IDLE:
            {
                State buffer_state = _requested_state;
                _requested_state = State::NONE;
                return buffer_state;
            }
            break;
        case State::LIGNEDROITE:
            {
                if(_parent_asser->get_consigne() == State::IDLE)
                {
                    // la consigne a ete remplie
                    return State::IDLE;
                }
                return State::NONE;
            }
            break;
        case State::VIRAGE:
            {
                if(_parent_asser->get_consigne() == State::IDLE)
                {
                    // la consigne a ete remplie
                    return State::IDLE;
                }
                return State::NONE;
            }
            break;
        default:
            return State::NONE;
            break;
    }
}

void StateMachine::_enter_state(State new_state, State old_state)
{
    switch(new_state)
    {
        case State::IDLE:
            _parent_asser->set_consigne(State::IDLE);
            break;
        case State::LIGNEDROITE:
            _parent_asser->set_consigne(State::LIGNEDROITE);
            break;
        case State::VIRAGE:
            _parent_asser->set_consigne(State::VIRAGE);
            break;
        default:
            break;
    }
}

void StateMachine::_exit_state(State new_state, State old_state)
{
    switch(old_state)
    {
        case State::IDLE:

            break;
        case State::LIGNEDROITE:

            break;
        case State::VIRAGE:
            
            break;
        default:
            break;
    }
}

void StateMachine::_request_state(State query)
{
    _requested_state = query;
}

State StateMachine::_get_state()
{
    return _state;
}