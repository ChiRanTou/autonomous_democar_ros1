# 自动驾驶控制算法学习demo
## 环境配置：ROS 1
## 前期准备: 安装qpOASES解算器
1.下载qpOASES
```
git clone https://github.com/coin-or/qpOASES.git
```
2.进入文件夹并进行安装
```
cd qpOASES
cmake .
make
sudo make install
```
安装成功后可以在/usr/loca/include中看到
## 使用DEMO
### 编译demo
1. 进入demo文件夹
```
cd autonomous_democar_ros1
```
2. 编译
```
catkin_make
source devell/setup.bash
```
### 使用demo
1. 打开小车模型
```
roslaunch democar_description democar_gazebo.launch
```
2. 让小车动起来
2.1. 通过键盘操控(可选)
重新打开一个终端，并进入文件夹
```
cd autonomous_democar_ros1
source devel/setup.bash
```
然后启动键盘控制功能
```
rosrun democar_telop democar_telop_node
```
2.2 通过控制算法控制小车
同样按照2.1先打开终端并索引到文件夹

2.2.1. 纯跟踪算法
```
roslaunch democar_core democar_purepursuit.launch
```
2.2.2. LQR算法 - 运动学模型
```
roslaunch democar_core democar_lqr_kinematics.launch
```
2.2.3. LQR算法 - 动力学模型
```
roslaunch democar_core democar_lqr_dynamics.launch
```
2.2.4. MPC算法
```
roslaunch democar_core democar_mpc.launch
```
路径可在各算法源码中进行更换，默认读取path2.csv
