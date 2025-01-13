#!/bin/bash
sudo echo "start"
ros2 service call /tita3037322/robot_inertia_calculator/enable_calculate std_srvs/srv/SetBool "{data: true}"
sudo insmod /home/robot/gs_usb.ko
# 定义变量
IMAGE="modify_airbot"
CONTAINER_NAME="airbot"
WORKSPACE_DIR="/home/robot/manipulator_ws/airbot_ws"
SSH_DIR="$HOME/.ssh"

# 启动容器并执行命令
docker run -it --rm --name $CONTAINER_NAME --privileged --cap-add=SYS_PTRACE \
    -v $SSH_DIR:/root/.ssh \
    --network=host \
    -v $WORKSPACE_DIR:/workspace \
    --workdir /workspace \
    $IMAGE /bin/bash -c "
    echo 'Running inside Docker container...';
    # 这里可以写要在容器内部执行的命令
    cd /workspace;
    bash /workspace/can_up.bash; # 启动can1
    bash /workspace/ros_run.bash; # 启动ros以及bridge
    # 添加更多命令
    echo 'Task completed in container';
    # /bin/bash  # 保持shell以便你能继续操作
    "

# 注意：如果你想容器退出后不保留它，加上--rm选项
# --rm 会自动删除容器，避免残留
