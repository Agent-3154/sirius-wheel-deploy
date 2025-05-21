#ifndef SIM_MEMORY_SHARE_DATA_H
#define SIM_MEMORY_SHARE_DATA_H

struct Robot_Control_Motor_Cmd {
    double q[12];
    double qd[12];
    double kp[12];
    double kd[12];
    double tau_ff[12];
};

struct Robot_State{
  double quat[4];
  double gyro[3];
  double acc[3];
  double q[12];
  double qd[12];
};

struct Sim_Plot {
    mjtNum foot_pos_des_[4][3]{};
    mjtNum foot_pos_[4][3]{};
    mjtNum pos_des_[3]{};
    mjtNum pos_[3]{};
};
#endif