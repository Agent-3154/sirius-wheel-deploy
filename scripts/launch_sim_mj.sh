#!/bin/bash
#gnome-terminal --tab
#sudo iox-roudi

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo LD_LIBRARY_PATH=. ldconfig

sudo LD_LIBRARY_PATH=. ./../build/test_repo/sim_test

#sudo LD_LIBRARY_PATH=. ./../build/test_repo/sim_test --sim_model_name=../robot/robot_model/belt/scene.xml --sub_imu=false --sub_usb2can=false --use_rc=true --run_type=3