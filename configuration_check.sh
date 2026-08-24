#!/bin/bash
# configuration_check.sh - 验证 ros2_control 配置是否正确
# 用法: ./configuration_check.sh

echo "=========================================="
echo "   ros2_control 配置验证工具"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 函数：打印状态
print_status() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}[✓]${NC} $2"
    else
        echo -e "${RED}[✗]${NC} $2"
    fi
}

# 1. 检查 URDF 轮子关节
echo "=== 1. 检查 URDF 轮子关节 ==="
URDF_JOINTS=$(ros2 param get /robot_state_publisher robot_description 2>/dev/null | grep -o 'joint name="[^"]*_wheel_joint"' | sed 's/joint name="//;s/"//')

if echo "$URDF_JOINTS" | grep -q "left_front_wheel_joint"; then
    print_status 0 "找到 left_front_wheel_joint"
else
    print_status 1 "未找到 left_front_wheel_joint"
fi

if echo "$URDF_JOINTS" | grep -q "left_back_wheel_joint"; then
    print_status 0 "找到 left_back_wheel_joint"
else
    print_status 1 "未找到 left_back_wheel_joint"
fi

if echo "$URDF_JOINTS" | grep -q "right_front_wheel_joint"; then
    print_status 0 "找到 right_front_wheel_joint"
else
    print_status 1 "未找到 right_front_wheel_joint"
fi

if echo "$URDF_JOINTS" | grep -q "right_back_wheel_joint"; then
    print_status 0 "找到 right_back_wheel_joint"
else
    print_status 1 "未找到 right_back_wheel_joint"
fi

echo ""

# 2. 检查 diff_drive_controller 参数（如果运行中）
echo "=== 2. 检查 diff_drive_controller 参数 ==="
if ros2 node list | grep -q "diff_drive_controller"; then
    echo "控制器节点正在运行"
    
    LEFT_WHEELS=$(ros2 param get /diff_drive_controller left_wheel_names 2>/dev/null)
    RIGHT_WHEELS=$(ros2 param get /diff_drive_controller right_wheel_names 2>/dev/null)
    WHEEL_SEP=$(ros2 param get /diff_drive_controller wheel_separation 2>/dev/null)
    WHEEL_RAD=$(ros2 param get /diff_drive_controller wheel_radius 2>/dev/null)
    
    echo ""
    echo "左侧轮子: $LEFT_WHEELS"
    echo "右侧轮子: $RIGHT_WHEELS"
    echo "轮距: $WHEEL_SEP m"
    echo "轮径: $WHEEL_RAD m"
    echo ""
    
    # 验证关节名
    if echo "$LEFT_WHEELS" | grep -q "left_front_wheel_joint"; then
        print_status 0 "左轮名称正确"
    else
        print_status 1 "左轮名称错误 (应该是 left_front_wheel_joint, left_back_wheel_joint)"
    fi
    
    # 验证轮距
    if echo "$WHEEL_SEP" | grep -q "0.4"; then
        print_status 0 "轮距正确 (0.4m)"
    else
        print_status 1 "轮距错误 (应该是 0.4m，当前: $WHEEL_SEP)"
    fi
    
    # 验证轮径
    if echo "$WHEEL_RAD" | grep -q "0.1"; then
        print_status 0 "轮径正确 (0.1m)"
    else
        print_status 1 "轮径错误 (应该是 0.1m，当前: $WHEEL_RAD)"
    fi
else
    echo -e "${YELLOW}[!]${NC} diff_drive_controller 未运行，跳过参数检查"
fi

echo ""

# 3. 检查 joint_states
echo "=== 3. 检查 /joint_states 话题 ==="
if ros2 topic list | grep -q "/joint_states"; then
    echo "/joint_states 话题存在"
    
    # 获取关节名列表
    JS_NAMES=$(ros2 topic echo /joint_states --once 2>/dev/null | grep "name:" -A 5 | grep -o "'[^']*'" | tr '\n' ' ')
    echo "关节名: $JS_NAMES"
    
    if echo "$JS_NAMES" | grep -q "left_front_wheel_joint"; then
        print_status 0 "joint_states 包含 left_front_wheel_joint"
    else
        print_status 1 "joint_states 不包含 left_front_wheel_joint"
    fi
else
    print_status 1 "/joint_states 话题不存在"
fi

echo ""

# 4. 检查 TF 树
echo "=== 4. 检查 TF 树 ==="
if ros2 run tf2_tools view_frames 2>/dev/null; then
    if [ -f "frames.pdf" ]; then
        echo "TF 树已生成到 frames.pdf"
        
        # 检查关键 TF
        if ros2 run tf2_ros tf2_echo base_link left_front_wheel 2>/dev/null | grep -q "Translation"; then
            print_status 0 "base_link → left_front_wheel TF 存在"
        else
            print_status 1 "base_link → left_front_wheel TF 缺失"
        fi
    fi
else
    echo -e "${YELLOW}[!]${NC} 无法生成 TF 树"
fi

echo ""

# 5. 总结
echo "=========================================="
echo "   配置验证总结"
echo "=========================================="
echo ""
echo "关键检查项："
echo "  1. URDF 关节名是否为 *_front_wheel_joint 和 *_back_wheel_joint"
echo "  2. diff_drive_controller 的 left_wheel_names 和 right_wheel_names 是否匹配"
echo "  3. wheel_separation 是否为 0.4m（匹配 URDF 的 ±0.2 y坐标）"
echo ""
echo "如有错误，请检查以下文件："
echo "  - myrobot_gazebo/src/robot/urdf.xacro"
echo "  - myrobot_gazebo/config/ros2_controller.yaml"
echo ""

# 6. 里程计快速测试建议
echo "=========================================="
echo "   里程计测试建议"
echo "=========================================="
echo ""
echo "直线测试 (验证 vx 和轮径):"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' -r 10"
echo "  # 运行 5 秒，应该前进 1m"
echo "  ros2 topic echo /odom --field pose.pose.position.x"
echo ""
echo "旋转测试 (验证 wz 和轮距):"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.314}}' -r 10"
echo "  # 运行 10 秒，应该旋转约 180° (π rad)"
echo "  ros2 topic echo /odom --field pose.pose.orientation"
echo ""