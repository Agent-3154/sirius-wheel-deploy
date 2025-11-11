#include "observation.h"
#include <iostream>

class ProjectedGravity : public Observation {
    private:
        Eigen::MatrixXf projected_gravity_buffer;
        int steps;
        int interval;
    public:
        ProjectedGravity(int steps, int interval) : steps(steps), interval(interval) {
            this->projected_gravity_buffer = Eigen::MatrixXf(3, steps * interval);
            this->projected_gravity_buffer.setZero();
        }
        void update(FSM_State_RL *fsm_state_rl) {
            for (int i = this->projected_gravity_buffer.cols() - 1; i > 0; i--) {
                this->projected_gravity_buffer.col(i) = this->projected_gravity_buffer.col(i - 1);
            }
            auto quat = fsm_state_rl->quat_;
            Eigen::Quaterniond quat_eigen(quat[0], quat[1], quat[2], quat[3]);
            auto projected_gravity = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -1));
            this->projected_gravity_buffer.col(0) = projected_gravity.cast<float>();
        }
        
        int get_size() {
            return 3 * this->steps;
        }

        Eigen::VectorXf compute() {
            // Return interleaved values similar to projected_gravity_buf[::interval]
            Eigen::VectorXf result(3 * this->steps);
            for (int i = 0; i < this->steps; i++) {
                int col_idx = i * this->interval;
                result.segment(i * 3, 3) = this->projected_gravity_buffer.col(col_idx);
            }
            return result;
        }
};


class JointPosMultistep : public Observation {
    private:
        Eigen::MatrixXf joint_pos_buffer;
        int steps;
        int interval;
    public:
        const int num_joints = 12;

        JointPosMultistep(int steps, int interval) : steps(steps), interval(interval) {
            this->joint_pos_buffer = Eigen::MatrixXf(num_joints, steps * interval);
            this->joint_pos_buffer.setZero();
        }
        
        void update(FSM_State_RL *fsm_state_rl) {
            // roll over the buffer
            for (int i = this->joint_pos_buffer.cols() - 1; i > 0; i--) {
                this->joint_pos_buffer.col(i) = this->joint_pos_buffer.col(i - 1);
            }
            Eigen::VectorXf jpos_leg_filtered = fsm_state_rl->raw_jpos_buffer_.rowwise().mean().eval();
            this->joint_pos_buffer.col(0) = jpos_leg_filtered;
        }
        
        int get_size() {
            return num_joints * this->steps;
        }

        Eigen::VectorXf compute() {
            // Return interleaved values similar to joint_pos_buf[::interval]
            Eigen::VectorXf result(num_joints * this->steps);
            for (int i = 0; i < this->steps; i++) {
                int col_idx = i * this->interval;
                result.segment(i * num_joints, num_joints) = this->joint_pos_buffer.col(col_idx);
            }
            return result;
        }
};


class JointVelMultistep : public Observation {
    private:
        Eigen::MatrixXf joint_vel_buffer;
        int steps;
        int interval;
    public:
        const int num_joints = 12;

        JointVelMultistep(int steps, int interval) : steps(steps), interval(interval) {
            this->joint_vel_buffer = Eigen::MatrixXf(num_joints, steps * interval);
            this->joint_vel_buffer.setZero();
        }
        void update(FSM_State_RL *fsm_state_rl) {
            // roll over the buffer
            for (int i = this->joint_vel_buffer.cols() - 1; i > 0; i--) {
                this->joint_vel_buffer.col(i) = this->joint_vel_buffer.col(i - 1);
            }
            Eigen::VectorXf jvel_leg_filtered = fsm_state_rl->raw_jvel_buffer_.rowwise().mean().eval();
            this->joint_vel_buffer.col(0) = jvel_leg_filtered;
        }

        int get_size() {
            return num_joints * this->steps;
        }

        Eigen::VectorXf compute() {
            // Return interleaved values similar to joint_vel_buf[::interval]
            Eigen::VectorXf result(num_joints * this->steps);
            for (int i = 0; i < this->steps; i++) {
                int col_idx = i * this->interval;
                result.segment(i * num_joints, num_joints) = this->joint_vel_buffer.col(col_idx);
            }
            return result;
        }
};


class PrevActions : public Observation {
    private:
        Eigen::MatrixXf prev_actions_;
        int steps_;
    public:
        const int num_joints = 12;
        PrevActions(int steps) : steps_(steps) {
        
        }
        void update(FSM_State_RL *fsm_state_rl) {
            this->prev_actions_ = fsm_state_rl->prev_actions_.leftCols(this->steps_).eval();
        }

        int get_size() {
            return num_joints * this->steps_;
        }

        Eigen::VectorXf compute() {
            return Eigen::Map<Eigen::VectorXf>(this->prev_actions_.data(), this->num_joints * this->steps_);
        }
};

