
# startup
``` bash 
chmod +x *.bash # for executable
./docker_run.bash
# in docker container
source /opt/ros/noetic/setup.bash
catkin_make
./can_up.bash # success if print "can1 up success"
./ros_run.bash # launch ros_interface of airbot
# if in terminal print such as : 
# terminate called after throwing an instance of 'std::runtime_error'
# what():  AIRBOT Play needs to be calibrated. Please run airbot_auto_set_zero or airbot_set_zero
# please kill with `Ctrl+C` and run with `airbot_set_zero -m can1` to set arm to zero position
# more information of airbot please refer to https://discover-robotics.github.io/docs/latest/AIRBOT-Play/tutorials/env/

```