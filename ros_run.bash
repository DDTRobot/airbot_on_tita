#!/bin/bash


# 启动一个新的 tmux 会话
tmux new-session -d -s airbot_session

# 在第一个窗口中运行终端1的命令
tmux send-keys -t airbot_session 'cd /workspace/airbot_ws' C-m
tmux send-keys -t airbot_session 'source ${ROS1_PATH}' C-m
tmux send-keys -t airbot_session 'source devel/setup.bash' C-m
tmux send-keys -t airbot_session 'roslaunch ros_interface airbot_arm.launch end_mode:=gripper' C-m

# 创建第二个窗口并运行终端2的命令
tmux new-window -t airbot_session
tmux send-keys -t airbot_session:1 'cd /workspace/airbot_ws' C-m
tmux send-keys -t airbot_session:1 'source ${ROS1_PATH}' C-m
tmux send-keys -t airbot_session:1 'source ${ROS2_PATH}' C-m
tmux send-keys -t airbot_session:1 'source ${ROS_BRIDGE}' C-m
tmux send-keys -t airbot_session:1 'export ROS_MASTER_URI=http://localhost:11311' C-m
tmux send-keys -t airbot_session:1 'ros2 run ros1_bridge dynamic_bridge' C-m

# 切换到第二个窗口
tmux select-window -t airbot_session:0

# 附加到 tmux 会话
tmux attach -t airbot_session
