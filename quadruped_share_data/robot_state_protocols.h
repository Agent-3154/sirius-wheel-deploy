#ifndef ROBOT_STATE_PROTOCOLS_H
#define ROBOT_STATE_PROTOCOLS_H

struct Robot_Control_Motor_Cmd {
    double q[18];
    double qd[18];
    double kp[18];
    double kd[18];
    double tau_ff[18];
};

struct Robot_State {
    double quat[4];
    double gyro[3];
    double acc[3];
    double q[18];
    double qd[18];
    int a;
    int b;
    int x;
    int y;
    int lb;
    int rb;
    int start;
    int back;
    int select;
    int home;
    int lo;
    int ro;
    int lx;
    int ly;
    int rx;
    int ry;
    int lt;
    int rt;
    int xx;
    int yy;
};

struct Sim_Plot {
    double foot_pos_des_[4][3]{};
    double foot_pos_[4][3]{};
    double pos_des_[3]{};
    double pos_[3]{};
};
#endif
