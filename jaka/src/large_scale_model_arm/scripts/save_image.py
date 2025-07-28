#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraSaver:
    def __init__(self, save_interval=0.5):
        # 初始化 ROS 节点
        rospy.init_node('camera_saver', anonymous=True)

        # 创建 CvBridge 对象
        self.bridge = CvBridge()

        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber('/camera/wrist/rgb', Image, self.image_callback)

        # 图像计数器
        self.image_counter = 0

        # 设置保存间隔（秒）
        self.save_interval = save_interval

        # 定时器，每隔 save_interval 秒调用 save_image 方法
        self.timer = rospy.Timer(rospy.Duration(self.save_interval), self.save_image)

        # 存储图像的路径
        self.save_path = "/home/q/Go2/saved_image_/"

    def image_callback(self, msg):
        try:
            # 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def save_image(self, event):
        try:
            # 检查是否有有效的图像
            if hasattr(self, 'cv_image'):
                # 构造保存的文件名
                filename = self.save_path + str(self.image_counter) + '.jpg'

                # 保存图像到文件
                cv2.imwrite(filename, self.cv_image)

                rospy.loginfo("Saved image to %s", filename)
                self.image_counter += 1
            else:
                rospy.logwarn("No image received yet, skipping saving.")
        except Exception as e:

            rospy.logerr("Error saving image: %s", str(e))

    def run(self):
        # 保持 ROS 节点运行
        rospy.spin()

if __name__ == '__main__':
    try:
        # 创建 CameraSaver 对象，设置每 5 秒保存一次图像
        saver = CameraSaver(save_interval=0.5)
        saver.run()
    except rospy.ROSInterruptException:
        pass
