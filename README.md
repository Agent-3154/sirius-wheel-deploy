### Introduction
This is a simple FSM based on Mujoco Simulator

### Dependency
* Eigen 3.4.0
* LCM 1.5.0
* libusb
### Installation
First, setup the enviroment.
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

#### If you run the project in Mujoco Simulator, you can run the following command:
* Go to the ergo_controller/mj_ctrl folder.
```bash
mkdir build && cd build
cmake ..
make
cd ./../scripts
bash launch_sim_mj.sh
```
Then you will get a mujoco window, click the Start_Runner button to start the simulation.
##### Pls note that if you want to change the robot(For now BELT, chaojigo, Go1). You need to follow the following steps:
* Change the definitions in ergo_controller/mj_ctrl/CMakeLists.txt.
* make again.

---

### Debug the project
* All data can be wathced in the lcm channel both for Gazebo and Mujoco.
Go to the mj_ctrl/scripts folder.
```bash
bash launch_lcm.sh
```
* Plot data in Mujoco Simulator: Press _Back_ button while simulation is running.

### Remote Controller:
*After clicking the start in simulator*:
* STAND: LB + A
* SitDown: LB + Logitech
* Passive: LB + X
* RL MODE: LB + START when stand


