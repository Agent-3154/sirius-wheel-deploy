#!/bin/bash

## super sirius
sudo ifconfig enp3s0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp3s0
#sudo ifconfig lo multicast
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo LD_LIBRARY_PATH=. ldconfig

sudo LD_LIBRARY_PATH=. ./../build/test_repo/test_ros