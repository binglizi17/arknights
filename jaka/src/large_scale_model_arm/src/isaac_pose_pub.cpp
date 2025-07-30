/*

*  2025无人系统具身智能算法挑战赛
*  版权所有。
   *
*  这个源码是比赛源码；您可以对其进行修改。
*  根据 GNU 通用公共许可证的条款对其进行修改。
   *
*  发布此源码是为了希望能对参赛者有所帮助。
*  但无任何保证；甚至没有隐含的保证。
*  适销性或特定用途的适用性。请参阅GNU协议。

*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

using json = nlohmann::json;

class ArmEndPosePublisher
{
public:
  ArmEndPosePublisher(ros::NodeHandle &nh) : nh_(nh)
  {
    nh_.param("fx", fx_, 317.70819899);
    nh_.param("fy", fy_, 317.52182399);
    nh_.param("cx", cx_, 323.07963808);
    nh_.param("cy", cy_, 181.78095701);

    R_ << 0.94991166, -0.07666748, 0.30296855,
          0.10819967, 0.99016619, -0.08867778,
         -0.29319052, 0.11701716, 0.94886580;
    T_ << 0.50721949, 0.22229940, 0.33096606; 

    gripper_offset_ << 0.0, 0.0, 0.0;

    image_sub_ = nh_.subscribe("/Jaka/camera_rgb", 1, &ArmEndPosePublisher::imageCallback, this);
    bbox_sub_ = nh_.subscribe("camera_pose", 10, &ArmEndPosePublisher::bboxCallback, this);
    pose_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("arm_end_pose_quat", 10);

    ROS_INFO("ArmEndPosePublisher node started.");
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      latest_cv_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void bboxCallback(const std_msgs::String::ConstPtr &msg)
  {
    if (latest_cv_image.empty()) {
      ROS_WARN("No image received yet.");
      return;
    }

    json j;
    try {
      j = json::parse(msg->data);
    } catch (json::parse_error &e) {
      ROS_ERROR("JSON parse error: %s", e.what());
      return;
    }

    if (!j.is_array() || j.empty()) {
      ROS_ERROR("Invalid JSON format");
      return;
    }

    int color_width = latest_cv_image.cols;
    int color_height = latest_cv_image.rows;

    for (auto &item : j)
    {
      if (!item.contains("bbox") || !item.contains("center_depth")) {
        ROS_WARN("Missing required fields in JSON item");
        continue;
      }

      auto &bbox = item["bbox"];
      double x1 = bbox["x1"].get<double>();
      double y1 = bbox["y1"].get<double>();
      double x2 = bbox["x2"].get<double>();
      double y2 = bbox["y2"].get<double>();
      double depth_m = item["center_depth"].get<double>();

      double center_u_norm = (x1 + x2) / 2.0;
      double center_v_norm = (y1 + y2) / 2.0;
      double u = (center_u_norm / 1000.0) * color_width;
      double v = (center_v_norm / 1000.0) * color_height;

      double camera_x = (u - cx_) * depth_m / fx_;
      double camera_y = (v - cy_) * depth_m / fy_;
      Eigen::Vector3d camera_point(camera_x, camera_y, depth_m);

      Eigen::Vector3d robot_point = R_ * camera_point + T_;

      Eigen::Vector3d end_effector_point = robot_point + gripper_offset_;

      double roll = -3.13, pitch = 0.031, yaw = -0.19;

      Eigen::Quaterniond quat(
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      );

      std_msgs::Float64MultiArray pose_msg;
      pose_msg.data = { end_effector_point.x(), end_effector_point.y(), end_effector_point.z(),
                        quat.x(), quat.y(), quat.z(), quat.w() };
      pose_pub_.publish(pose_msg);

      ROS_INFO("Published pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
               end_effector_point.x(), end_effector_point.y(), end_effector_point.z(),
               quat.x(), quat.y(), quat.z(), quat.w());
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber bbox_sub_, image_sub_;
  ros::Publisher pose_pub_;
  cv::Mat latest_cv_image;

  double fx_, fy_, cx_, cy_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d T_, gripper_offset_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_end_pose_publisher");
  ros::NodeHandle nh;
  ArmEndPosePublisher node(nh);
  ros::spin();
  return 0;
}
