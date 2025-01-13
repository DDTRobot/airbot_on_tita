#!/bin/bash

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