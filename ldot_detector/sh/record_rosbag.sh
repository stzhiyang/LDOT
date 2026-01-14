#!/bin/bash

# ROS Bag 录制脚本
# 用于录制 LDOT 检测器相关话题

# 设置录制参数
BAG_NAME="ldot_test_$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$HOME/rosbags"
DURATION=60  # 默认录制时长（秒），0表示手动停止

# 创建保存目录
mkdir -p "$BAG_DIR"

# 定义要录制的话题
TOPICS=(
    "/livox/lidar"              # 激光雷达点云
    "/livox/imu"
    "/mavros/imu"
    "/Odometry"                 # 里程计（同步触发）
    "/Odom_high_freq"           # 高频里程计（运动补偿）
)

# 打印录制信息
echo "=========================================="
echo "ROS Bag 录制脚本"
echo "=========================================="
echo "保存路径: $BAG_DIR/$BAG_NAME.bag"
echo "录制话题:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo "录制时长: ${DURATION}秒 (0=手动停止)"
echo "=========================================="
echo ""

# 检查 ROS 是否运行
if ! rostopic list &> /dev/null; then
    echo "错误: ROS Master 未运行，请先启动 roscore"
    exit 1
fi

# 检查话题是否存在
echo "检查话题可用性..."
MISSING_TOPICS=()
for topic in "${TOPICS[@]}"; do
    if ! rostopic info "$topic" &> /dev/null; then
        MISSING_TOPICS+=("$topic")
    fi
done

if [ ${#MISSING_TOPICS[@]} -gt 0 ]; then
    echo "警告: 以下话题当前不可用:"
    for topic in "${MISSING_TOPICS[@]}"; do
        echo "  - $topic"
    done
    echo ""
    read -p "是否继续录制? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "录制已取消"
        exit 0
    fi
fi

# 开始录制
echo "开始录制..."
cd "$BAG_DIR"

if [ "$DURATION" -eq 0 ]; then
    # 手动停止模式
    echo "按 Ctrl+C 停止录制"
    rosbag record -O "$BAG_NAME" "${TOPICS[@]}"
else
    # 定时停止模式
    echo "将在 ${DURATION} 秒后自动停止"
    rosbag record --duration="$DURATION" -O "$BAG_NAME" "${TOPICS[@]}"
fi

# 录制完成
echo ""
echo "=========================================="
echo "录制完成!"
echo "文件保存在: $BAG_DIR/$BAG_NAME.bag"
echo "文件大小: $(du -h "$BAG_DIR/$BAG_NAME.bag" | cut -f1)"
echo "=========================================="

# 显示 bag 信息
echo ""
echo "Bag 文件信息:"
rosbag info "$BAG_NAME.bag"
