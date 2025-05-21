### Introduction
This is a simple project based on Mujoco Simulator/Gazebo Simulator and ROS.

### Installation
First, setup the enviroment.
* Denpendencies
  * Eigen 3.4.0
  * LCM 1.5.0
  * libusb
  * iceoryx: https://github.com/eclipse-iceoryx/iceoryx/tree/main

* Libraries
```bash
sudo apt-get install libglfw3-dev libboost-all-dev
```
* Setup the GCC-11
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get install libgl1-mesa-dev libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev ninja-build
sudo apt update
sudo apt-get install gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60 --slave /usr/bin/g++ g++ /usr/bin/g++-11 
```

---

### Running the project
#### Remote FSM:
* LB + A = STAND
* LB + X = PASSIVE
* LB + Y = RL_WALK
* LB + B = ?
* LB + Logitech = SITDOWN
#### Run the Project
*Simulate your controller in Mujoco Simulator*
```angular2html
sudo iox-roudi
bash ./scripts/launch_sim_mj.sh # in a new terminal
bash ./scripts/launch_sim_ctrl.sh # in a new terminal
```
Watch the data in the lcm channel.
```bash
bash ./scripts/launch_lcm.sh
```




