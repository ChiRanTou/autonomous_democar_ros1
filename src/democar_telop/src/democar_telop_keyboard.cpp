/*键盘控制小车*/

//导入c++自带头文件
#include <iostream>
#include <string>
#include <math.h>

//导入ros相关头文件
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

//导入其他文件
#include "keyboard_control.h"

using namespace std;
using std::placeholders::_1;

/*
    Q   W   E   :   left-forward    forward     right-forward
    A   S   D   :   left-turn        stop       right-turn  
    Z   X   C   :   left_backward   backward    right-backward
*/
//设置按键
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_j 0x6A
#define KEYCODE_k 0x6B


class DemocarTelopKeyBoard
{
    public:
        //构造函数
        DemocarTelopKeyBoard();

    private:
        //键盘控制
        void democarTelopKey();
        //显示菜单
        void showMenu();
        //小车制动
        void stopCar();
        //获取按键信息
        void get_key();

        //申明节点
        ros::NodeHandle nh_;
        //申明发布小车运动指令的话题
        ros::Publisher pub_cmd;

};

/*  对对象中的函数进行定义  */
//构造函数定义
DemocarTelopKeyBoard::DemocarTelopKeyBoard()
{
    ROS_INFO("Starting up democar telop keyboard controller");
    //显示菜单
    showMenu();
    
    pub_cmd = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

    //执行小车键盘控制
    democarTelopKey();

    //只有在需要找键盘点时才取消注释
    //get_key();
}

//显示菜单
void DemocarTelopKeyBoard::showMenu()
{
    cout << "-----------------------------------------------------------------" << endl;
    cout << "|               Welcome to democar telop keyboard               |" << endl;
    cout << "-----------------------------------------------------------------" << endl;
    cout << "|    Q   W   E   |   left-forward    forward     right-forward  |" << endl;
    cout << "|    A   S   D   |   left-turn        stop       right-turn     |" << endl;
    cout << "|    Z   X   C   |   left_backward   backward    right-backward |" << endl;
    cout << "-----------------------------------------------------------------" << endl;
    cout << endl;
    cout << "press ctrl+c to quit" << endl;
}

//停车制动
void DemocarTelopKeyBoard::stopCar()
{
    //设置制动的数值
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;   //直行
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;  //转向
    //显示制动通知
    //RCLCPP_INFO(this->get_logger(),"Stopping the demo car...");
    //发布制动信息
    pub_cmd.publish(twist);
}

//键盘控制
void DemocarTelopKeyBoard::democarTelopKey()
{
    auto KBC = KeyboardCtrl();
    geometry_msgs::Twist twist;

    while(ros::ok())
    {
        //获得按键
        auto key = KBC.get_press_key();

        //按键判断
        switch (key)
        {
        case KEYCODE_W:     //前进
            twist.linear.x = 1.0;
            twist.angular.z = 0.0;
            cout << "Move forward" << endl;
            break;
        case KEYCODE_S:     //停止
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cout << "Stop" << endl;
            break;
        case KEYCODE_A:     //原地左转
            twist.linear.x = 0.0;
            twist.angular.z = 0.5;
            cout << "Left rotate" << endl;            
            break;
        case KEYCODE_D:     //原地右转
            twist.linear.x = 0.0;
            twist.angular.z = -0.5;
            cout << "Right rotate" << endl;            
            break;
        case KEYCODE_Q:     //左转弯
            twist.linear.x = 1.0;
            twist.angular.z = 0.5;
            cout << "Left forward turn" << endl;            
            break;
        case KEYCODE_E:     //右转弯
            twist.linear.x = 1.0;
            twist.angular.z = -0.5;
            cout << "Right forward turn" << endl;            
            break;
        case KEYCODE_Z:     //左后倒车
            twist.linear.x = -1.0;
            twist.angular.z = -0.5;
            cout << "Left backward turn" << endl;            
            break;
        case KEYCODE_X:     //往后倒车
            twist.linear.x = -1.0;
            twist.angular.z = 0.0;
            cout << "Move backward" << endl;  
            break;
        case KEYCODE_C:     //右后倒车
            twist.linear.x = -1.0;
            twist.angular.z = 0.5;
            cout << "Right backward turn" << endl;  
            break;
        default:
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            break;
        }
        pub_cmd.publish(twist);
    }
}

//获取按键代码
//平常是被注释掉
//只是需要知道新按键时才用
void DemocarTelopKeyBoard::get_key()
{
    auto KBC = KeyboardCtrl();
    auto key = KBC.get_press_key();
    ROS_INFO("get keyboard press 0x%02X \n",key);
}

//主程序入口
int main(int argc,char **argv)
{
    //ros初始化
    ros::init(argc,argv,"democar_telop_keyboard");

    DemocarTelopKeyBoard democarTelop;

    //spin
    ros::spin();
    
    //关闭ros
    ros::shutdown();

    return 0;
}