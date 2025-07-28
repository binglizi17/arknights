#!/home/q/anaconda3/envs/inference/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String 
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image
import sys
import torch
from transformers import AutoModel, AutoTokenizer
import threading
import numpy as np
rospy.loginfo("Using Python executable: %s", sys.executable)

class ImageProcessorNode:
    def __init__(self, default_bbox_prompt):
        self.model = None
        self.tokenizer = None

        # 保存最新的彩色图像，使用锁保护
        self.latest_cv_image = None
        self.frame_lock = threading.Lock()

        self.bbox_prompt = default_bbox_prompt
        self.prompt_lock = threading.Lock()

        self.new_bbox_request = False
        self.bbox_request = None  

        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher("camera_pose", String, queue_size=10)
        self.model_output_pub = rospy.Publisher("model_output", String, queue_size=10)  

        # 加载大模型
        model_file = '/home/q/model/FM9G4B-V'
        try:
            rospy.loginfo("正在从 %s 加载模型...", model_file)
            self.model = AutoModel.from_pretrained(
                model_file,
                trust_remote_code=True,
                attn_implementation='sdpa',
                torch_dtype=torch.bfloat16
            )
            self.model = self.model.eval().to(device='cuda', dtype=torch.bfloat16)
            self.tokenizer = AutoTokenizer.from_pretrained(model_file, trust_remote_code=True)
            rospy.loginfo("模型和tokenizer加载成功。")
        except Exception as e:
            rospy.logerr("模型加载失败: %s", e)
            self.model = None
            self.tokenizer = None

        # 仅在模型加载成功时订阅彩色图像
        if self.model is not None and self.tokenizer is not None:
            try:
                self.image_sub = rospy.Subscriber("/yolo_annotated_image", ROSImage, self.image_callback, queue_size=1)
                rospy.loginfo("成功订阅彩色摄像头话题。")
            except Exception as e:
                rospy.logerr("订阅彩色摄像头数据失败: %s", e)
        else:
            rospy.logerr("模型或tokenizer加载失败，无法订阅彩色图像数据。")

    def image_callback(self, data):
        """持续接收彩色摄像头数据，并保存最新一帧图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("彩色图像转换出错: %s", e)
            return

        with self.frame_lock:
            self.latest_cv_image = cv_image

    def process_latest_frame(self):
        """
        读取最新彩色图像，调用大模型进行处理，并展示图像
        """
        if not self.new_bbox_request:
            return

        with self.frame_lock:
            if self.latest_cv_image is None:
                rospy.logwarn("当前没有有效彩色图像帧，跳过处理。")
                return
            cv_image = self.latest_cv_image.copy()

        # 将BGR图像转换为RGB，并转换为PIL格式供大模型使用
        try:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)
        except Exception as e:
            rospy.logerr("彩色图像转换错误: %s", e)
            return

        with self.prompt_lock:
            current_bbox_prompt = self.bbox_prompt
        msgs = [{'role': 'user', 'content': [pil_image, current_bbox_prompt]}]
        try:
            model_res = self.model.chat(
                image=None,
                msgs=msgs,
                tokenizer=self.tokenizer
            )
            rospy.loginfo("模型处理结果:\n%s", model_res)

            model_output_msg = String()
            model_output_msg.data = str(model_res) 
            self.model_output_pub.publish(model_output_msg)
            rospy.loginfo("模型结果已发布到话题 'model_output'.")

        except Exception as e:
            rospy.logerr("调用大模型进行处理时出错: %s", e)
            return

    def command_input_thread(self):
        """
        监听终端输入指令：
        当输入新的请求时，更新提示内容并立即触发对最新图像的处理。
        """
        while not rospy.is_shutdown():
            try:
                # 等待终端输入新的请求提示
                new_prompt = input("\n请输入处理请求（输入后立即处理最新图像）：")
                if new_prompt.strip(): 
                    with self.prompt_lock:
                        self.bbox_prompt = new_prompt.strip()
                    self.new_bbox_request = True 
                    
                    rospy.loginfo("请求更新成功，开始处理最新图像...")
                    # 触发处理最新图像
                    self.process_latest_frame()
                else:
                    rospy.loginfo("输入为空，未更新请求。")
                    self.new_bbox_request = False
            except Exception as e:
                rospy.logerr("终端输入错误: %s", e)
            rospy.sleep(0.1)

def main():
    rospy.init_node('ros_image_processor', anonymous=True)

    default_bbox_prompt = "请处理图像并返回结果"

    ipn = ImageProcessorNode(default_bbox_prompt)


    thread = threading.Thread(target=ipn.command_input_thread)
    thread.daemon = True  
    thread.start()

    rospy.on_shutdown(lambda: cv2.destroyAllWindows())

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("检测到键盘中断，正在关闭节点。")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
