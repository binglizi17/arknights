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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

class CmdVelSplitter
{
public:
    CmdVelSplitter()
    {
        ros::NodeHandle nh;
        pub_x_ = nh.advertise<std_msgs::Float32>("/cmd_vel_x", 10);
        pub_y_ = nh.advertise<std_msgs::Float32>("/cmd_vel_y", 10);
        pub_yaw_ = nh.advertise<std_msgs::Float32>("/cmd_vel_yaw", 10);
        sub_twist_ = nh.subscribe("/cmd_vel", 10, &CmdVelSplitter::twistCallback, this);
    }

private:
    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std_msgs::Float32 fx, fy, fyaw;
        fx.data = (msg->linear.x != 0.0) ? static_cast<float>(msg->linear.x) : 0.0f;
        fy.data = (msg->linear.y != 0.0) ? static_cast<float>(msg->linear.y) : 0.0f;
        fyaw.data = (msg->angular.z != 0.0) ? static_cast<float>(msg->angular.z) : 0.0f;
        pub_x_.publish(fx);
        pub_y_.publish(fy);
        pub_yaw_.publish(fyaw);
    }

    ros::Subscriber sub_twist_;
    ros::Publisher pub_x_, pub_y_, pub_yaw_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go2_cmd_vel");
    CmdVelSplitter splitter;
    ros::spin();
    return 0;
}
