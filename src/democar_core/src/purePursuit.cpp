//导入c++自带头文件
#include <iostream>
#include <math.h>
#include <vector>

//导入ros相关头文件
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//自定义文件
#include "loadPath.h"

using namespace std;

//纯跟踪算法
class PurePursuit
{
    public:
        //构造函数
        PurePursuit();

    private:
        //初始化信息
        void initSettings();
        //加载路径点信息
        void loadPathInfo();
        //发布全局路径点
        void pubPath();
        //当前位姿回调函数
        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
        //查找最近路径点下标
        int findCloesetPointIndex(const nav_msgs::Odometry::ConstPtr& odom);
        //计算前轮转角
        double calSteeringAngle(const double alpha,const double ld);

        //申明节点
        ros::NodeHandle nh_;

        //控制指令话题
        ros::Publisher pub_cmd;
        //全局路径话题
        ros::Publisher pub_path;
        //当前位姿话题
        ros::Subscriber sub_odom;

        //文件名称
        string pkg_name = "democar_core";
        string waypoints_name="path2.csv";
        string waypoints_path;

        //轴距
        double l;
        //起始预瞄距离
        double ld0;
        //预瞄系数
        double kv;
        //最大转向角
        double max_steer_angle;

        //全局路径点
        vector<double> xr;
        vector<double> yr;
        vector<double> yawr;
        vector<double> kappar;

        //定义全局路径信息
        nav_msgs::Path base_path;
};


PurePursuit::PurePursuit()
{
    ROS_INFO("Starting Pure Pursuit");
    
    //获取参数
    initSettings();
    //获取路径点
    loadPathInfo();
    //发布控制指令话题设置
    pub_cmd = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    //发布已走路径话题设置
    pub_path = nh_.advertise<nav_msgs::Path>("base_path",1);
    //订阅位姿话题设置
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("odom",1,bind(&PurePursuit::odom_callback,this,_1));

}

//获取参数
void PurePursuit::initSettings()
{
    //获取车辆参数
    nh_.getParam("/pure_pursuit/car_length",l);                         //车身轴距
    nh_.getParam("/pure_pursuit/default_preview_distance",ld0);         //初始预瞄距离
    nh_.getParam("/pure_pursuit/preview_ratio",kv);                     //预瞄距离系数
    nh_.getParam("/pure_pursuit/max_steer_angle",max_steer_angle);      //最大转向角

    max_steer_angle *= M_PI/180.0;    //角度转弧度
    //申明和初始化参数到对象变量中
    /*l = 0.4;
    ld0 = 0.5;
    kv = 0.1;
    max_steer_angle = 45.0 * M_PI/180.0;*/

    //获取路径点文件
    string pkg_path = ros::package::getPath(pkg_name);
    waypoints_path = pkg_path+"/waypoints/"+waypoints_name;

    ROS_INFO("waypoints file is loaded at directory: %s",waypoints_path.c_str());
}

//加载路径点信息
void PurePursuit::loadPathInfo()
{
    WaypointLoader wp(waypoints_path);
    //检查文件是否存在或是否为空
    bool isLoaded = wp.load_waypoints();
    if (!isLoaded)
    {
        ROS_ERROR("File is not exist or file is empty!");
        exit(1);
    }
    vector<vector<double>> wp_temp = wp.get_waypoints();
    for(vector<vector<double>>::const_iterator it=wp_temp.begin();it!=wp_temp.end();it++)
    {
        //获取路径点和位姿信息
        xr.push_back((*it)[0]);     //x
        yr.push_back((*it)[1]);     //y
        yawr.push_back((*it)[2]);   //yaw
        kappar.push_back((*it)[3]);      //kappa
    }
}

//发布全局路径点
void PurePursuit::pubPath()
{
    //申明一个poseStamped话题放置路径点
    geometry_msgs::PoseStamped pose;

    //设置全局路径的时间戳
    base_path.header.stamp = ros::Time::now();
    //设置全局路径的id
    base_path.header.frame_id = "odom";
    //设置路径点的时间戳
    base_path.header.stamp = ros::Time::now();
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

//当前位姿回调函数
//纯跟踪算法的主要接口
void PurePursuit::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    //发布全局路径点
    pubPath();

    //找到最近路径点的下标位置
    int index = findCloesetPointIndex(odom);

    //获取预瞄路径点上的坐标信息
    double x_prev = xr[index];
    double y_prev = yr[index];

    //从里程计获取小车的坐标和速度信息
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double v = odom->twist.twist.linear.x;
    //cout <<"x_prev = " << x_prev << ", y_prev = " << y_prev << endl;
    //cout <<"x = " << x << ", y = " << y << endl;

    //把四元数转换成欧拉角
    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w
    );

    double roll,pitch,yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);

    //计算alpha角（小车与预瞄点间的夹角，需要减去小车航向角进行纠正）
    double alpha = atan2(y_prev-y,x_prev-x)-yaw;
    //计算预瞄距离
    double ld = kv*v+ld0;
    //计算转向角
    double steer = calSteeringAngle(alpha,ld);
    //发布动作信息
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = steer;
    pub_cmd.publish(cmd_vel);
    ROS_INFO("Velocity = %f, Steering angle = %f",cmd_vel.linear.x,(cmd_vel.angular.z)*180.0/M_PI);
}

//查找最近路径点下标
int PurePursuit::findCloesetPointIndex(const nav_msgs::Odometry::ConstPtr& odom)
{   
    //位置索引
    int index;
    //与路径点距离
    vector<double> dist;
    //获取当前速度
    double v = odom->twist.twist.linear.x;
    //取出位置信息
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;

    //计算全局路径点到小车位置的距离
    for (int i =0;i<(int)xr.size();i++)
    {
        double dist_temp = pow(xr[i]-x,2) + pow(yr[i]-y,2);
        dist.push_back(dist_temp);
    }
    //获得最小值
    auto smallest = min_element(dist.begin(),dist.end());
    //获得最小值对应的下标
    index = distance(dist.begin(),smallest);

    //计算预瞄距离
    double ld = kv*v+ld0;
    double ld_now = 0;

    //遍历全局路径点
    //找到符合在预瞄距离上且不在最终点的位置下标
    //比较的是下一个路径点与当前最近路径点的距离
    while(ld_now<ld && index<=(int)xr.size())
    {
        double dx_ref = xr[index+1] - xr[index];
        double dy_ref = yr[index+1] - yr[index];
        ld_now += sqrt(pow(dx_ref,2)+pow(dy_ref,2));
        index++;
    }
  
    return index;
}

//计算前轮转角
double PurePursuit::calSteeringAngle(const double alpha,const double ld)
{
    //设置转向限制
    double steer = atan2(2*l*sin(alpha),ld);
    if (steer > max_steer_angle)
    {
        steer = max_steer_angle;
    }
    else if (steer < -max_steer_angle)
    {
        steer = -max_steer_angle;
    }

    //返回转向角
    return steer;
}

int main(int argc,char **argv)
{
    //初始化ros
    ros::init(argc,argv,"pure_pursuit");

    //创建纯跟踪控制器对象
    PurePursuit purepursuit;

    //spin
    ros::spin();

    //关闭
    ros::shutdown();

    return 0;
}