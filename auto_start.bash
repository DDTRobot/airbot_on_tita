#!/bin/bash

sudo insmod /home/robot/gs_usb.ko

# 检查 can1 接口状态
while true; do
    if ip link show can1 | grep -q "UP"; then
        echo "can1 up success"
        break
    else
        echo "can1 up fail, trying to up again..."
        sudo ip link set can1 up type can bitrate 1000000
        sleep 0.2  # 等待 1 秒后再检查
    fi
done
sleep 1

CONTAINER_NAME="airbot"

if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
    echo "容器 ${CONTAINER_NAME} 已经在运行。"
else
    echo "启动容器 ${CONTAINER_NAME} 并运行服务..."

    # 启动 Docker 容器并在其中执行命令
    docker run -d --rm --name ${CONTAINER_NAME} --privileged --cap-add=SYS_PTRACE -v $HOME/.ssh:/root/.ssh --network=host -v $(pwd):/workspace --workdir /workspace modify_airbot /bin/bash -c "tail -f /dev/null"
    # 例如：docker run -d --name ${CONTAINER_NAME} my_app_image /usr/bin/supervisord
fi

# docker exec ${CONTAINER_NAME} bash -c "
#     bash ./ros_run.bash; # 启动ros以及bridge
#     # 完成后输出消息
#     echo 'Task completed in container';
# "

docker exec ${CONTAINER_NAME} bash -c "
    bash ./ros_run.bash; # 启动ros以及bridge
    # 完成后输出消息
    echo 'Task completed in container';
"

# ROS1_PATH="/opt/ros/noetic/setup.bash"
# ROS2_PATH="/root/ros2_humble/install/setup.bash"
# ROS_BRIDGE="/root/ros1_bridge/install/setup.bash"
# docker exec  ${CONTAINER_NAME} bash -c "
#     cd /workspace/airbot_ws;
#     source ${ROS1_PATH};
#     source ${ROS2_PATH};
#     source ${ROS_BRIDGE};
#     export ROS_MASTER_URI=http://localhost:11311;
#     echo ${ROS_MASTER_URI};
#     ros2 run ros1_bridge dynamic_bridge " &

# docker exec  ${CONTAINER_NAME} bash -c "
#     cd /workspace/airbot_ws;
#     source ${ROS1_PATH};
#     source devel/setup.bash;
#     roslaunch ros_interface airbot_arm.launch end_mode:=none
# " 

