#!/usr/bin/env python3
# coding=utf-8

"""
采集 ROS 图像话题和机械臂的位姿数据，并保存到文件中。
订阅话题 "camera/color/image_raw" 获取图像。
订阅话题 "/Jaka/get_end_effector_pose" 获取末端位姿数据（geometry_msgs/PoseStamped）。
当按下 's' 键时，程序会保存当前图像及位姿信息。
按下 'q' 键退出程序。
"""

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np

image_save_path = "/home/q/jaka/src/hand_eye_calibrate/collect_data/"
count = 0

bridge = CvBridge()
latest_image = None
latest_pose = None  

def image_callback(msg):
    global latest_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        latest_image = cv_image
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

def pose_callback(msg):
    global latest_pose
    try:
        latest_pose = msg.pose
        rospy.loginfo("接收到末端位姿数据: position=(%f, %f, %f)",
                      latest_pose.position.x, latest_pose.position.y, latest_pose.position.z)
    except Exception as e:
        rospy.logerr("处理末端位姿数据时出错：%s", e)

def data_collect():
    global count, latest_image, latest_pose
    rospy.init_node('data_collect_node', anonymous=True)
    rospy.Subscriber("/Jaka/camera_rgb", Image, image_callback)
    rospy.Subscriber("/Jaka/get_end_effector_pose", PoseStamped, pose_callback)
    rospy.loginfo("已订阅话题：camera/color/image_raw 和 /Jaka/get_end_effector_pose")
    cv2.namedWindow('detection', cv2.WINDOW_NORMAL)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_image is not None:
            cv2.imshow("detection", latest_image)
        key = cv2.waitKey(1) & 0xFF  
        if key == ord('s'):
            rospy.loginfo("采集数据中……")
            if latest_pose is not None:
                pose = [
                    latest_pose.position.x, latest_pose.position.y, latest_pose.position.z,
                    latest_pose.orientation.x, latest_pose.orientation.y, latest_pose.orientation.z, latest_pose.orientation.w
                ]
                rospy.loginfo("获取到的机器人位姿：%s", pose)
                pose_str = ",".join(str(i) for i in pose)
            else:
                pose_str = "获取位姿失败"
            with open(f'{image_save_path}poses.txt', 'a+', encoding='utf-8') as f:
                f.write(pose_str + "\n")
            if latest_image is not None:
                image_filename = f'{image_save_path}{count}.jpg'
                cv2.imwrite(image_filename, latest_image)
                rospy.loginfo("图像已保存至 %s", image_filename)
            else:
                rospy.logwarn("当前无图像数据可保存！")
            count += 1
        elif key == ord('q'):
            rospy.loginfo("退出采集程序")
            break
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    data_collect()
