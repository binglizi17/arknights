#!/usr/bin/env python3
# coding=utf-8

"""
键盘控制机械臂关节角度示例程序：
1. 程序订阅 /Jaka/get_jointstate 获取当前机械臂关节状态（类型 sensor_msgs/JointState）。
2. 利用键盘输入更新关节角度，并发布到 /Jaka/set_jointstate。
3. 控制映射示例：
   主要关节（Joint01 ~ Joint06）：
       增加：q, w, e, r, t, y
       减少：a, s, d, f, g, h
   手指关节（finger1_joint 和 finger2_joint）：
       增加：u, i
       减少：j, k
4. 按 ESC 键退出程序。
"""

import rospy
import cv2
import numpy as np
import sys
from sensor_msgs.msg import JointState

# 全局变量保存当前关节角度
current_joint_angles = {}
# 订阅话题中的关节名称（按 rostopic echo 输出）
arm_joint_names = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06"]
finger_joint_names = ["finger1_joint", "finger2_joint"]
all_joint_names = arm_joint_names + finger_joint_names

default_angles = {name: 0.0 for name in all_joint_names}

# 键盘映射：键值 -> (关节名称, 角度变化)
# 主要关节
key_bindings = {
    ord('q'): ("Joint01", 0.1), ord('a'): ("Joint01", -0.1),
    ord('w'): ("Joint02", 0.1), ord('s'): ("Joint02", -0.1),
    ord('e'): ("Joint03", 0.1), ord('d'): ("Joint03", -0.1),
    ord('r'): ("Joint04", 0.1), ord('f'): ("Joint04", -0.1),
    ord('t'): ("Joint05", 0.1), ord('g'): ("Joint05", -0.1),
    ord('y'): ("Joint06", 0.1), ord('h'): ("Joint06", -0.1),
    # 手指关节（可以根据需要调整步长）
    ord('u'): ("finger1_joint", 0.1), ord('j'): ("finger1_joint", -0.1),
    ord('i'): ("finger2_joint", 0.1), ord('k'): ("finger2_joint", -0.1)
}

# def jointstate_callback(msg):
#     global current_joint_angles
#     # 以消息中的 joint name 为键，position 为值存入字典
#     for name, pos in zip(msg.name, msg.position):
#         current_joint_angles[name] = pos

def print_instructions():
    print("=============================================")
    print("键盘控制机械臂关节角度:")
    print("主要关节:")
    print("  Joint01 增加: q    减少: a")
    print("  Joint02 增加: w    减少: s")
    print("  Joint03 增加: e    减少: d")
    print("  Joint04 增加: r    减少: f")
    print("  Joint05 增加: t    减少: g")
    print("  Joint06 增加: y    减少: h")
    print("手指关节:")
    print("  finger1_joint 增加: u    减少: j")
    print("  finger2_joint 增加: i    减少: k")
    print("退出：按 ESC 键")
    print("=============================================")

def main():
    global current_joint_angles
    rospy.init_node('keyboard_joint_controller', anonymous=True)

    # 订阅机械臂当前关节状态
    # rospy.Subscriber("/Jaka/get_jointstate", JointState, jointstate_callback)
    # 发布关节状态消息到控制话题
    pub = rospy.Publisher("/Jaka/set_jointstate", JointState, queue_size=10)

    # 如果还未收到实际关节数据，使用默认值初始化
    current_joint_angles = default_angles.copy()

    # 创建一个简单窗口用于显示当前关节角度和捕获键盘输入
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.namedWindow("Joint Control", cv2.WINDOW_NORMAL)
    print_instructions()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # 在图像上显示当前关节角度
        display_img = img.copy()
        y0 = 30
        dy = 30
        for idx, name in enumerate(all_joint_names):
            angle = current_joint_angles.get(name, 0.0)
            text = f"{name}: {angle:.2f}"
            cv2.putText(display_img, text, (10, y0 + idx * dy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2)
        cv2.imshow("Joint Control", display_img)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC 键退出
            print("退出控制程序")
            break

        if key in key_bindings:
            joint_name, delta = key_bindings[key]
            # 更新当前关节角度
            current_angle = current_joint_angles.get(joint_name, 0.0)
            new_angle = current_angle + delta
            current_joint_angles[joint_name] = new_angle
            rospy.loginfo("更新 %s 角度为 %.2f", joint_name, new_angle)

            # 构造 JointState 消息并发布
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            # 保证消息中各个关节顺序与机器人期望一致
            msg.name = all_joint_names
            msg.position = [current_joint_angles.get(name, 0.0) for name in all_joint_names]
            pub.publish(msg)

        rate.sleep()

    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
