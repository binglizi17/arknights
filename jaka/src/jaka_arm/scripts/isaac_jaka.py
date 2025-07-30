"""

*  2025无人系统具身智能算法挑战赛
*  版权所有。
   *
*  这个源码是比赛源码；您可以对其进行修改。
*  根据 GNU 通用公共许可证的条款对其进行修改。
   *
*  发布此源码是为了希望能对参赛者有所帮助。
*  但无任何保证；甚至没有隐含的保证。
*  适销性或特定用途的适用性。请参阅GNU协议。

"""
#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped

def arm_pose_callback(msg):
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

    desired = PoseStamped()
    desired.header.stamp = rospy.Time.now()
    desired.header.frame_id = msg.header.frame_id 

    desired.pose = msg.pose  

    set_pose_pub.publish(desired)
    rospy.loginfo("Published to /Jaka/set_end_effector_pose: Position[%.3f, %.3f, %.3f]",
                  desired.pose.position.x,
                  desired.pose.position.y,
                  desired.pose.position.z)

def main():
    rospy.init_node("arm_end_pose_relay", anonymous=True)

    rospy.Subscriber("arm_end_pose_quaternion", PoseStamped, arm_pose_callback)

    global set_pose_pub
    set_pose_pub = rospy.Publisher("/Jaka/set_end_effector_pose", PoseStamped, queue_size=10)

    rospy.loginfo("Node started: relaying from 'arm_end_pose_quaternion' to '/Jaka/set_end_effector_pose'")
    rospy.spin()

if __name__ == "__main__":
    main()
