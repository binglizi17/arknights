#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped

def arm_pose_callback(msg):
    # 打印接收到的末端位姿
    rospy.loginfo("Received arm_end_pose_quaternion:")
    rospy.loginfo("  Position:   [%.3f, %.3f, %.3f]",
                  msg.pose.position.x,
                  msg.pose.position.y,
                  msg.pose.position.z)
    rospy.loginfo("  Orientation:[%.3f, %.3f, %.3f, %.3f]",
                  msg.pose.orientation.x,
                  msg.pose.orientation.y,
                  msg.pose.orientation.z,
                  msg.pose.orientation.w)

    # 构造并发布给机械臂的目标位姿
    desired = PoseStamped()
    desired.header.stamp = rospy.Time.now()
    desired.header.frame_id = msg.header.frame_id  # 保持坐标系一致

    # 如果需要对姿态做偏移，可以在这里修改 desired.pose.position 或 desired.pose.orientation
    desired.pose = msg.pose  # 直接复用

    set_pose_pub.publish(desired)
    rospy.loginfo("Published to /Jaka/set_end_effector_pose: Position[%.3f, %.3f, %.3f]",
                  desired.pose.position.x,
                  desired.pose.position.y,
                  desired.pose.position.z)

def main():
    rospy.init_node("arm_end_pose_relay", anonymous=True)

    # 订阅 arm_end_pose_quaternion 话题
    rospy.Subscriber("arm_end_pose_quaternion", PoseStamped, arm_pose_callback)

    # 发布到 Jaka 机械臂控制接口
    global set_pose_pub
    set_pose_pub = rospy.Publisher("/Jaka/set_end_effector_pose", PoseStamped, queue_size=10)

    rospy.loginfo("Node started: relaying from 'arm_end_pose_quaternion' to '/Jaka/set_end_effector_pose'")
    rospy.spin()

if __name__ == "__main__":
    main()
