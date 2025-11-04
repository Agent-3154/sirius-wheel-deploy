#include "observation.h"


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
            this->projected_gravity_buffer.col(0) = fsm_state_rl->projected_gravity_.cast<float>();
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
            this->joint_pos_buffer.col(0) = fsm_state_rl->obs_jpos_buffer_.col(0);
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
        const int num_joints = 4;

        JointVelMultistep(int steps, int interval) : steps(steps), interval(interval) {
            this->joint_vel_buffer = Eigen::MatrixXf(num_joints, steps * interval);
            this->joint_vel_buffer.setZero();
        }
        void update(FSM_State_RL *fsm_state_rl) {
            // roll over the buffer
            for (int i = this->joint_vel_buffer.cols() - 1; i > 0; i--) {
                this->joint_vel_buffer.col(i) = this->joint_vel_buffer.col(i - 1);
            }
            this->joint_vel_buffer.col(0) = fsm_state_rl->obs_jvel_buffer_.col(0);
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
    public:
        void update(FSM_State_RL *fsm_state_rl) {
            this->prev_actions_ = fsm_state_rl->prev_actions_;
        }

        int get_size() {
            return 2 * 16;
        }

        Eigen::VectorXf compute() {
            return Eigen::Map<Eigen::VectorXf>(this->prev_actions_.data(), this->prev_actions_.size());
        }
};

