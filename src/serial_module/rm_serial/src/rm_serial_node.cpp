#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <serial/serial.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>


serial::Serial ser;
bool if_over = false;
float aim_x = 0.0;
float aim_y = 0.0;
float self_x = 0.0;
float self_y = 0.0;


void if_over_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    self_x = msg->pose.pose.position.x;
    self_y = msg->pose.pose.position.y;
    float dis_x = self_x - aim_x;
    float dis_y = self_y - aim_y;

    if (dis_x*dis_x + dis_y*dis_y > 2)
        if_over = false;
    if (dis_x*dis_x + dis_y*dis_y < 0.17)
    {
        if_over = true;
        ROS_INFO("--------Achieve!!!!--------");    
    }
}

void receive_callback(ros::Publisher& pub) {
    // ROS_INFO("----ava:%d----", ser.available());
    if (ser.available() >= 1) { // 确保至少有12个字节可用
        std::string head = ser.read(1); // 读取12个字节的数据
        if (head[0] == 'D') { // 检查数据格式是否正确
            // 解析数据
            std::string data = ser.read(11);
            if (data[10] != 'H')
                return;
            int x_sign = (data[0] == '1') ? 1 : -1;
            aim_x = std::stoi(data.substr(1, 4)) * 0.01 * x_sign;
            int y_sign = (data[5] == '1') ? 1 : -1;
            aim_y = std::stoi(data.substr(6, 4)) * 0.01 * y_sign;
	        ROS_INFO("get a goal point: %s", data.c_str());
            // 创建并发布PointStamped消息
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = aim_x; 
            pose_msg.pose.position.y = aim_y; 
            pose_msg.pose.position.z = 0.0;
            pose_msg.pose.orientation.x  = 0.0;
            pose_msg.pose.orientation.y = 0.0;
            pose_msg.pose.orientation.z = 0.0;
            pose_msg.pose.orientation.w = 1.0; 
            pub.publish(pose_msg);
        }
    }
}

std::string formatData(double value) {
    std::stringstream ss;
    int intValue = static_cast<int>(value * 10000);  // 将值转换为整数（毫米）
    ss << std::fixed << std::setfill('0') << std::setw(5) << std::abs(intValue);
    return ss.str().substr(0,5);  // 确保字符串长度为5
}

void write_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    // ROS_INFO("Write_success");
    std::string data;
    data += "A";

    // 处理linear.x的符号位和数据
    data += (msg->twist.linear.y < 0) ? "1" : "0";
    data += formatData(msg->twist.linear.y);

    // 处理linear.y
    data += (msg->twist.linear.x < 0) ? "1" : "0";
    data += formatData(msg->twist.linear.x);

    // 处理angular.z
    // data += (msg->twist.angular.z < 0) ? "1" : "0";
    // data += formatData(0);
    
    // LZT：如果到达终点，倒数第二位为Y，否则为N
    if (if_over)
    	data += "1";
    else
    	data += "0"; 
    
    data += "a";
    // 打印data到终端
    ROS_INFO("Data to be sent: %s", data.c_str());
    ROS_INFO("X:%f", msg->twist.linear.x);
    ROS_INFO("Y:%f", msg->twist.linear.y);

    ser.write(data);
}

int main (int argc, char** argv){
    ROS_INFO("Serial begin");
    ros::init(argc, argv, "rm_serial");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    std::string serial_port;
    nh.param<std::string>("/rm_serial/serial_port", serial_port, "/dev/ttyUSB0");

    // 打开串口
    try{
        ROS_INFO_STREAM("Serial Port: " << serial_port);
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port " << serial_port);
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // 订阅 /cmd_vel
    ros::Subscriber write_sub = nh.subscribe("/cmd_vel", 1000, write_callback);

    // 订阅自己位姿
    ros::Subscriber self_pos = nh.subscribe("/state_estimation", 1000, if_over_callback);

    // 订阅/if_over
    // ros::Subscriber judgeIf_over = nh.subscribe("/if_over", 1000, judge_callback);
    // 创建并初始化发布器
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10); 
    
    ROS_INFO("Action!");
    while(ros::ok())
    {
        receive_callback(pub);
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();
}
