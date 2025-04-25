#!/bin/bash

sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp3s0
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo LD_LIBRARY_PATH=. ldconfig

#sudo LD_LIBRARY_PATH=. ./../build/test_repo/sim_test --sim_model_name=../robot/robot_model/belt/scene.xml --sub_imu=true --sub_usb2can=true --use_rc=false --run_type=0
sudo LD_LIBRARY_PATH=. ./../build/test_repo/show_robot