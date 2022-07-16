/*lqr controller for kinematics model*/

//导入C++自带头文件
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

//导入ros相关文件
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//自定义头文件
#include "loadPath.h"

using namespace std;
using namespace Eigen;

//lqr控制器对象
class LqrController
{
    public:
        //构造函数
        LqrController();
    private:
        //初始化参数
        void initParameters();
        //加载路径点信息
        void loadPathInfo();
        //找到最近的路径点
        int findCLoestIndex();
        //误差计算模块
        void calStateError(const int index);
        //计算状态空间矩阵
        void calStateMatrix(const int index);
        //lqr迭代器
        void lqrSolver();
        //最终控制
        double calSteering(const int index);
        //发布全局路径
        void pubPath();
        //归一化角度
        double normalizeAngle(const double angle);
        //lqr控制函数主体
        double lqrContoll();

        //里程计回调函数
        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

        //申明一个节点
        ros::NodeHandle nh_;
        //申明控制指令发布者
        ros::Publisher pub_cmd;
        //申明全局路径发布者
        ros::Publisher pub_path;
        //申明里程计订阅者
        ros::Subscriber sub_odom;

        //小车物理属性
        //车长
        double l_;
        //控制时间域
        double ts_;
        //最大转向角
        double max_steer_angle;

        //状态空间矩阵
        //状态大小
        const int basic_state_size = 3;
        //状态空间矩阵
        Eigen::MatrixXd a_mat_;
        //状态离散矩阵
        Eigen::MatrixXd a_mat_d_;
        //控制矩阵
        Eigen::MatrixXd b_mat_;
        //控制离散矩阵
        Eigen::MatrixXd b_mat_d_;
        //gain矩阵(LQR迭代出来的最后输出)
        Eigen::MatrixXd k_;
        //控制权重矩阵
        Eigen::MatrixXd r_;
        //状态权重矩阵
        Eigen::MatrixXd q_;
        //状态矩阵
        Eigen::MatrixXd state_mat;

        //状态误差
        double x_err;           //x-xr
        double y_err;           //y-yr
        double headiang_err;    //yaw-yawr

        //lqr设置
        //最大迭代次数
        int lqr_max_iteration = 200;
        //最小迭代误差
        double lqr_eps;

        //路径点容器
        vector<double> xr;
        vector<double> yr;
        vector<double> yawr;
        vector<double> kappar;
        vector<double> thetar;
        double vr;

        //自身车辆位姿
        double x_car;
        double y_car;
        double yaw_car;
        double linear_velocity;
        double heading_rate;

        //路径点数量
        int numOfWaypoints;

        //文件名称
        string pkg_name = "democar_core";
        string waypoints_name = "path2.csv";
        string waypoints_path;

        //定义全局路径信息
        nav_msgs::Path base_path;
};

//构造函数
LqrController::LqrController()
{
    ROS_INFO("Starting up lqr controller");
    
    //初始化各种参数
    initParameters();
    //读取路径点信息
    loadPathInfo();
    //全局路径发布者
    pub_path = nh_.advertise<nav_msgs::Path>("base_path",1);
    //控制指令发布者
    pub_cmd = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    //里程计订阅者
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("odom",1,bind(&LqrController::odom_callback,this,_1));
}

//初始化参数
void LqrController::initParameters()
{
    
    //从yaml中获取车辆信息
    nh_.getParam("/lqr_controller/car_length",l_);                         //车长
    nh_.getParam("/lqr_controller/control_period",ts_);                    //控制时间
    nh_.getParam("/lqr_controller/max_steer_angle",max_steer_angle);       //最大转向角
    nh_.getParam("/lqr_controller/lqr_epsilon",lqr_eps);                   //迭代误差
    nh_.getParam("/lqr_controller/reference_velocity",vr);                 //参考速度

    //初始化各种矩阵
    a_mat_ = MatrixXd::Identity(basic_state_size,basic_state_size);
    a_mat_d_ = MatrixXd::Identity(basic_state_size,basic_state_size);
    b_mat_ = MatrixXd::Zero(basic_state_size,2);
    b_mat_d_ = MatrixXd::Zero(basic_state_size,2);
    k_ = MatrixXd::Zero(2,basic_state_size);
    r_ = MatrixXd::Identity(2,2);
    q_ = MatrixXd::Identity(basic_state_size,basic_state_size);
    q_(0,0) = 30;
    q_(1,1) = 30;
    q_(2,2) = 30;
    state_mat = MatrixXd::Zero(basic_state_size,1);

    //角度换算 degree -> rad
    max_steer_angle *= M_PI/180.0;

    //获取路径文件名称
    string pkg_path = ros::package::getPath(pkg_name);
    waypoints_path = pkg_path+"/waypoints/"+waypoints_name;

    ROS_INFO("waypoints file is loaded at directory: %s",waypoints_path.c_str());

}

//加载路径点信息
void LqrController::loadPathInfo()
{
    //创建路径加载器
    WaypointLoader wploader(waypoints_path);
    //获取读取是否成功的结果
    bool isFileLoaded = wploader.load_waypoints();
    if(!isFileLoaded)
    {
        ROS_ERROR("File is not exist or file is empty!");
        exit(1);
    }
    //读取路径点信息数据
    vector<vector<double>> wp = wploader.get_waypoints();
    for(vector<vector<double>>::const_iterator it=wp.begin();it!=wp.end();it++)
    {
        xr.push_back((*it)[0]);
        yr.push_back((*it)[1]);
        yawr.push_back((*it)[2]);
        kappar.push_back((*it)[3]);
        thetar.push_back(atan((*it)[3]*l_));
    }
    numOfWaypoints = (int)xr.size();
}
//找到最近的路径点
/*
    x:小车x坐标
    y:小车y坐标
    return：最近路径点下标
*/
int LqrController::findCLoestIndex()
{
    
    vector<double> dist; 
    //计算小车与路径点的距离
    for(int i=0;i<numOfWaypoints;i++)
    {
        dist.push_back(pow(xr[i]-x_car,2)+pow(yr[i]-y_car,2));
    }
    //获得最小值的位置指针
    auto smallest = min_element(dist.begin(),dist.end());
    //把位置指针转换成下标
    int index = distance(dist.begin(),smallest);
    return index;
}

//误差计算模块
/*
    index: 最近规划点下标
    x：小车x坐标
    y：小车y坐标
    yaw：小车航向角
*/
void LqrController::calStateError(const int index)
{
    //x误差
    x_err = x_car - xr[index];    
    //y误差
    y_err = y_car - yr[index];
    //航向误差
    headiang_err = normalizeAngle(yaw_car - yawr[index]);

    //把误差放入矩阵中
    state_mat(0,0) = x_err;
    state_mat(1,0) = y_err;
    state_mat(2,0) = headiang_err;
}

//计算空间状态矩阵
void LqrController::calStateMatrix(const int index)
{   
    //状态矩阵
    /*
        [1,0,-t*v*sin(yawr);
        0,1,t*v*cos(yawr);
        0,0,1]
    */
    a_mat_(0,2) = -ts_ * vr * sin(yawr[index]);
    a_mat_(1,2) = ts_ * vr * cos(yawr[index]);
    a_mat_d_ = a_mat_;
    //控制矩阵
    /*
        [t*cos(yawr),0;
        t*sin(yawr),0;
        t*tan(yawr)/l,t*v/(l*(cos(yawr))^2)]
    */
    b_mat_(0,0) = ts_ * cos(yawr[index]);
    b_mat_(1,0) = ts_ * sin(yawr[index]);
    b_mat_(2,0) = ts_ * tan(yawr[index]) / l_;
    b_mat_(2,1) = ts_ * vr / l_ / pow(cos(thetar[index]),2);
    //离散化处理
    b_mat_d_ = b_mat_;
}

//lqr迭代器
void LqrController::lqrSolver()
{
    //设置初始p矩阵
    Eigen::MatrixXd p_;
    Eigen::MatrixXd p_old_ = q_;
    //设置一个初始误差值(取一个无限大的值)
    double error = std::numeric_limits<double>::max();
    int i = 0;

    //迭代求解p矩阵直到最小
    while (i++ < lqr_max_iteration && error>lqr_eps)
    {
        p_ = a_mat_d_.transpose() * p_old_ * a_mat_d_ 
            - a_mat_d_.transpose() * p_old_ * b_mat_d_ * (r_ + b_mat_d_.transpose() * p_old_ * b_mat_d_).inverse() * b_mat_d_.transpose() * p_old_ * a_mat_d_
            + q_;
        error = fabs((p_-p_old_).maxCoeff());
        p_old_ = p_;
    }
    k_ = (r_ + b_mat_d_.transpose() * p_ * b_mat_d_).inverse() * b_mat_d_.transpose() * p_ * a_mat_d_;
}


//最终控制
double LqrController::calSteering(const int index)
{
    Eigen::MatrixXd temp_mat = -k_ * state_mat;
    double steering_angle = temp_mat(1,0) + thetar[index]; 
 
    //限幅
    if(steering_angle >= max_steer_angle)
    {
        steering_angle = max_steer_angle;
    }
    else if (steering_angle <= - max_steer_angle)
    {
        steering_angle = - max_steer_angle;
    }
    return steering_angle;
}

//lqr控制函数主体
double LqrController::lqrContoll()
{
    //计算最近规划点并返回其下标
    int index = findCLoestIndex();
    //计算状态误差
    calStateError(index);
    //计算状态空间矩阵
    calStateMatrix(index);
    //进行lqr迭代求最小代价下的control gain “k”
    lqrSolver();
    //求最终的横向控制率
    double steering = calSteering(index);

    return steering;
}

//发布全局路径点
void LqrController::pubPath()
{
    //申明一个poseStamped话题放置路径点
    geometry_msgs::PoseStamped pose;

    //设置全局路径的时间戳
    base_path.header.stamp = ros::Time::now();
    //设置全局路径的id
    base_path.header.frame_id = "odom";
    //设置路径点的时间戳
    pose.header.stamp = ros::Time::now();
    //设置路径点的id
    pose.header.frame_id = "odom";
    //遍历读取出来的路径点
    //并放置到pose中
    for(int i=0;i<(int)xr.size();i++)
    {   
        //保存位置信息
        pose.pose.position.x = xr[i];
        pose.pose.position.y = yr[i];
        pose.pose.position.z = 0.0;

        //保存方向信息
        tf2::Quaternion q;
        q.setRPY(0,0,yawr[i]);
    
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        //把路径点一个个放到全局路径中
        base_path.poses.push_back(pose);
    }

    //发布全局路径
    pub_path.publish(base_path);
}

//归一化角度
double LqrController::normalizeAngle(const double angle)
{
    //对角度取模
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if(a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;   
}

//里程计回调函数
void LqrController::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{

    //发布路径点
    pubPath();

    //获取当前小车的位姿
    x_car = odom_msg->pose.pose.position.x;         //x
    y_car = odom_msg->pose.pose.position.y;         //y
    tf2::Quaternion q_heading(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    //航向角
    //yaw_car已经在前面定义了
    double roll_car,pitch_car;
    tf2::Matrix3x3 m_heading(q_heading);
    m_heading.getRPY(roll_car,pitch_car,yaw_car);
    //速度
    double vx = odom_msg->twist.twist.linear.x;
    double vy = odom_msg->twist.twist.linear.y;
    linear_velocity = sqrt(pow(vx,2)+pow(vy,2));
    //航向率
    heading_rate = odom_msg->twist.twist.angular.z;
    //lqr控制
    //获得最终的转向角度
    double steering = lqrContoll();
    //发布动作信息
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vr;
    cmd_vel.angular.z = steering;
    pub_cmd.publish(cmd_vel);

    ROS_INFO("Velocity = %f, Steering angle = %f",cmd_vel.linear.x,(cmd_vel.angular.z)*180.0/M_PI);
}

//主程序接口
int main(int argc,char **argv)
{
    //初始化ros
    ros::init(argc,argv,"lqr_controller");

    //创建lqr控制器对象
    LqrController lqrcontroller;

    //启动节点
    ros::spin();

    //关闭节点
    ros::shutdown();

    return 0;

}