#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash # change this if your workspace is named differently

cd ~/ros2_ws/src/robot_sensor_state_checks/robot_sensorstate_checks/robot_sensorstate_checks

# run test files with pytest
python3 -m pytest test_sensor_topics.py test_state_topics.py -v 

