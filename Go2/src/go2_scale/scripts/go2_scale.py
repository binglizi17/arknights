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
#!/home/q/anaconda3/envs/inference/bin/python
# -*- coding: utf-8 -*-
import rospy, sys, threading, json, traceback, numpy as np, cv2
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from PIL import Image

import torch
from transformers import AutoModel, AutoTokenizer


def ensure_model_loaded(func):
    def wrapper(self, *args, **kw):
        if self.model is None or self.tokenizer is None:
            rospy.logwarn("模型未加载，跳过调用。")
            return
        return func(self, *args, **kw)
    return wrapper


class ImageProcessorNode:
    def __init__(self, default_bbox_prompt="请处理图像并返回结果"):
        rospy.loginfo("Python exec: %s", sys.executable)

        self.model, self.tokenizer = None, None
        self._load_model("/home/q/model/FM9G4B-V")

        self.bridge            = CvBridge()
        self.latest_cv_image   = None           
        self.current_map_image = None           
        self.frame_lock        = threading.Lock()
        self.map_lock          = threading.Lock()

        self.model_output_pub = rospy.Publisher("model_output", String, queue_size=10)
        self.nav_goal_pub     = rospy.Publisher("llm_nav_goal", String, queue_size=10)
        self.line_ctrl_pub    = rospy.Publisher("/line_follow_control", String, queue_size=10)

        rospy.Subscriber("/camera/front",        ROSImage,       self._camera_front_cb, queue_size=1)
        rospy.Subscriber("/yolo_annotated_image", ROSImage,       self._yolo_image_cb,   queue_size=1)
        rospy.Subscriber("/map",                 OccupancyGrid,  self._map_cb)

        threading.Thread(target=self._command_thread, daemon=True).start()
        rospy.on_shutdown(lambda: rospy.loginfo("Shutting down node ..."))

    def _load_model(self, model_dir):
        try:
            rospy.loginfo("加载模型 %s ...", model_dir)
            self.model = (AutoModel.from_pretrained(
                model_dir, trust_remote_code=True,
                attn_implementation="sdpa",
                torch_dtype=torch.bfloat16
            ).eval().to("cuda", dtype=torch.bfloat16))
            self.tokenizer = AutoTokenizer.from_pretrained(model_dir, trust_remote_code=True)
            rospy.loginfo("模型 & tokenizer 就绪。")
        except Exception as e:
            rospy.logerr("模型加载失败: %s\n%s", e, traceback.format_exc())

    def _camera_front_cb(self, msg):
        self._update_latest_frame(msg)

    def _yolo_image_cb(self, msg):
        self._update_latest_frame(msg)

    def _update_latest_frame(self, ros_img):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            with self.frame_lock:
                self.latest_cv_image = cv_img
        except CvBridgeError as e:
            rospy.logerr("CvBridge 转换错误: %s", e)

    def _map_cb(self, data):
        try:
            w, h = data.info.width, data.info.height
            arr  = np.array(data.data, dtype=np.int8).reshape((h, w))
            gray = np.zeros((h, w), dtype=np.uint8)
            gray[arr == 100] = 255
            with self.map_lock:
                self.current_map_image = Image.fromarray(gray)
        except Exception as e:
            rospy.logerr("地图处理错误: %s\n%s", e, traceback.format_exc())

    @ensure_model_loaded
    def _process_line_command(self, user_cmd: str):
        if "停止" in user_cmd:
            act = "stop"
        else:
            act = "start"
        self.line_ctrl_pub.publish(String(data=act))
        rospy.loginfo("巡线控制发布: %s", act)

    @ensure_model_loaded
    def _process_nav_command(self, user_cmd: str):
        with self.map_lock:
            if self.current_map_image is None:
                rospy.logwarn("暂无地图，无法导航。")
                return
            map_img = self.current_map_image

        kb = """
        已知导航点:
        - "通道/障碍": (6.03, 1.62)
        - "手榴弹桌子": (0.03, 1.54)
        - "手电筒桌子": (5.28, 3.65)
        - "烟雾弹桌子": (0.05,6.99)
        - "终点": (2.86, 8.24)
        """

        prompt = (f"你是机器人导航助手。\n{kb}\n"
                  f"指令:除了终点朝向为0,其他朝向(yaw)统一使用1.57 {user_cmd}\n"
                  "只返回 JSON，如 {\"x\":1.0,\"y\":2.0,\"yaw\":1.57}")

        try:
            res = self.model.chat(
                image=map_img,
                msgs=[{"role":"user", "content": prompt}],
                tokenizer=self.tokenizer
            )
            txt = str(res).strip()
            if "```json" in txt:
                txt = txt.split("```json")[1].split("```")[0].strip()

            rospy.loginfo("导航决策: %s", txt)
            goal = json.loads(txt)
            self.nav_goal_pub.publish(String(data=json.dumps(goal)))
            self.model_output_pub.publish(String(data=f"导航结果: {txt}"))
        except Exception as e:
            rospy.logerr("导航解析失败: %s\n%s", e, traceback.format_exc())

    @ensure_model_loaded
    def _custom(self, prompt: str):
        with self.frame_lock:
            cv_img = None if self.latest_cv_image is None else self.latest_cv_image.copy()

        pil_img = None
        if cv_img is not None:
            pil_img = Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        else:
            rospy.logwarn("当前无相机帧，改用纯文本模式。")

        try:
            res = self.model.chat(
                image=pil_img,
                msgs=[{"role": "user", "content": prompt}],
                tokenizer=self.tokenizer
            )
            out = str(res).strip()
            self.model_output_pub.publish(String(data=out))
            rospy.loginfo("模型输出发布。")
        except Exception as e:
            rospy.logerr("视觉/抓取处理失败: %s\n%s", e, traceback.format_exc())

    def _command_thread(self):
        grab_kw = ["抓取", "拿取", "拾取", "位置", "识别"]
        line_kw = ["巡线", "沿线", "停止巡线"]
        nav_kw  = ["导航", "穿过", "终点", "桌子", "手榴弹", "烟雾弹", "手电", "通道", "障碍"]

        while not rospy.is_shutdown():
            try:
                cmd = input("\n请输入指令: ").strip()
                if not cmd:
                    continue

                if any(k in cmd for k in grab_kw):
                    rospy.loginfo("→ 抓取/视觉")
                    self._custom(cmd)
                elif any(k in cmd for k in line_kw):
                    rospy.loginfo("→ 巡线")
                    self._process_line_command(cmd)
                elif any(k in cmd for k in nav_kw):
                    rospy.loginfo("→ 导航")
                    self._process_nav_command(cmd)
                else:
                    rospy.loginfo("→ 默认（视觉）")
                    self._custom(cmd)

                rospy.sleep(0.05)
            except Exception as e:
                rospy.logerr("输入线程错误: %s\n%s", e, traceback.format_exc())


if __name__ == '__main__':
    rospy.init_node("ros_image_processor", anonymous=True)
    ImageProcessorNode()
    rospy.spin()
