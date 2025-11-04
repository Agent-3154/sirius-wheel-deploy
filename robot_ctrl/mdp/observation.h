#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <eigen3/Eigen/Dense>
#include "../FSM/FSM_State_RL.h"

class Observation {
    public:
        virtual void update(FSM_State_RL *fsm_state_rl) = 0;
        virtual Eigen::VectorXf compute() = 0;
        virtual int get_size() = 0;
};

#endif