#!/bin/bash

## my laptop file
sudo ifconfig enp0s31f6 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp0s31f6
#sudo ifconfig lo multicast
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo LD_LIBRARY_PATH=. ldconfig

sudo LD_LIBRARY_PATH=. ./../build/test_repo/test_hardware
#sudo LD_LIBRARY_PATH=. ./../build/test_repo/test_hardware --alg_model_name=../robot/robot_model/unitree_go1/scene.xml --launch_imu=false --launch_usb2can=false --launch_rc=true --run_type=