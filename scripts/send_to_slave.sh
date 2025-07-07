#!/bin/bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/..
rm -rf mj_robot_software
mkdir mj_robot_software

cp -r ${DIR}/../build mj_robot_software/build
find . -name \*.so* -exec cp {} ./mj_robot_software/build \;
cp -r ${DIR}/../config mj_robot_software
cp -r ${DIR}/../robot mj_robot_software
cp -r ${DIR}/../models mj_robot_software
cp -r ${DIR}/../onnxruntime-linux-x64-1-16-1 mj_robot_software
cp -r ${DIR}/launch_real_robot.sh mj_robot_software/build

DATE=$(date +"%Y%m%d%H%M")

# test n100 ip
#scp -r mj_robot_software cuhk@192.168.123.26:~/
# super sirius
scp -r mj_robot_software sirius-wheel@192.168.123.25:~
# real_nuc ip
#scp -r mj_robot_software lrl@192.168.123.12:~/
rm -rf ${DIR}/../mj_robot_software



