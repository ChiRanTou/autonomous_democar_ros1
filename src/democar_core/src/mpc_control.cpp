/*mpc controller for dynamics model*/

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

//导入外部头文件
#include "qpOASES.hpp"


using namespace std;
using Matrix = Eigen::MatrixXd;

//lqr控制器对象
class MpcController
{
    public:
        //构造函数
        MpcController();
        //析构函数
        ~MpcController(){};
    private:
        //初始化参数
        void initParameters();
        //加载路径点信息
        void loadPathInfo();
        //找到最近的路径点
        int findCloestIndex(const double x,const double y);
        //误差计算模块
        void computeStateError(const double x,
                                const double y, 
                                const double v, 
                                const double heading, 
                                const double heading_rate, 
                                const int index);
        //计算状态空间矩阵
        void computeStateMatrix(const double linear_velocity);
        //mpc求解
        double mpcSolver();
        //qp解算器
        //返回控制量
        double solveQP(const Matrix& H_matrix,
                              const Matrix& G_matrix, 
                              const Matrix& inequality_matrix, 
                              const Matrix& inequality_lower_boundary_matrix,
                              const Matrix& inequality_upper_boundary_matrix,
                              const Matrix& lower_bound, 
                              const Matrix& upper_bound, 
                              const int numOfIter);
        //前馈控制
        double feedForwardControl(const double v, const double kappa);
        //最终控制
        double computeSteering(const double x,
                                const double y,
                                const double v,
                                const double heading, 
                                const double heading_rate);
        //发布全局路径
        void pubPath();
        //归一化角度
        double normalizeAngle(const double angle);

        //里程计回调函数
        void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

        //申明节点对象
        ros::NodeHandle nh_;
        //申明控制指令发布者
        ros::Publisher pub_cmd;
        //申明全局路径发布者
        ros::Publisher pub_path;
        //申明里程计订阅者
        ros::Subscriber sub_odom;

        //小车物理属性
        //控制时间域
        double ts_;
        //左前轮质量 
        double mass_front_left_wheel;
        //右前轮质量
        double mass_front_right_wheel;
        //左后轮质量
        double mass_rear_left_wheel;
        //右后轮质量
        double mass_rear_right_wheel;
        //车身质量
        double mass_body;
        //整车质量
        double mass_;
        //车辆轴长
        double wheel_base;
        //前轮轴长
        double lf_;
        //后轮轴长
        double lr_;
        //前轮侧偏刚度
        double cf_;
        //后轮侧偏刚度
        double cr_;
        //z轴旋转惯量
        double izz_;
        //最大转向角
        double max_steer_angle;
        //期望速度
        double v_ref;
        //预测步长
        double horizon_;
        //mpc最大迭代步长
        double numOfIter;

        //状态空间矩阵
        //状态大小
        //ed,ed_dot,e_psi,e_psi_dot
        const int basic_state_size = 4;
        //控制量大小
        const int basic_control_size = 1;
        //控制权重矩阵
        Matrix r_;
        //状态权重矩阵
        Matrix q_;
        //状态矩阵
        Matrix state_mat_;
        //控制矩阵
        Matrix control_mat_;
        //状态空间矩阵
        Matrix a_mat_;
        //离散状态空间矩阵
        Matrix a_mat_d_;
        //控制空间矩阵
        Matrix b_mat_;
        //离散控制空间矩阵
        Matrix b_mat_d_;
        //扰动空间矩阵
        Matrix c_mat_;
        //离散扰动空间矩阵
        Matrix c_mat_d_;
        
        //状态误差
        double lateral_error;           //ed
        double heading_error;           //e_psi
        double lateral_error_rate;      //ed_dot
        double heading_error_rate;      //e_psi_dot

        //路径点容器
        vector<double> xr;
        vector<double> yr;
        vector<double> yawr;
        vector<double> kappar;

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
MpcController::MpcController()
{
    ROS_INFO("Starting up MPC controller");

    //初始化各类参数
    initParameters();
    //加载路径点信息
    loadPathInfo();
    //设置订阅者和发布者
    pub_path = nh_.advertise<nav_msgs::Path>("base_path",1);                    //路径发布者
    pub_cmd = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);                //指令发布者
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("odom",1,bind(&MpcController::odom_callback,this,_1));        //里程计订阅者
}

//初始化各类参数
void MpcController::initParameters()
{
    //从yaml中获取车辆信息
    nh_.getParam("/mpc_controller/car_length",wheel_base);                                                   //车身轴距
    nh_.getParam("/mpc_controller/front_left_wheel_mass",mass_front_left_wheel);                             //左前轮质量
    nh_.getParam("/mpc_controller/front_right_wheel_mass",mass_front_right_wheel);                           //右前轮质量
    nh_.getParam("/mpc_controller/rear_left_wheel_mass",mass_rear_left_wheel);                               //左后轮质量
    nh_.getParam("/mpc_controller/rear_right_wheel_mass",mass_rear_right_wheel);                             //右后轮质量
    nh_.getParam("/mpc_controller/body_mass",mass_body);                                                     //车身质量
    nh_.getParam("/mpc_controller/front_wheel_stiffiness",cf_);                                              //前轮侧偏刚度
    nh_.getParam("/mpc_controller/rear_wheel_stiffiness",cr_);                                               //后轮侧偏刚度
    nh_.getParam("/mpc_controller/control_period",ts_);                                                      //控制时间
    nh_.getParam("/mpc_controller/max_steer_angle",max_steer_angle);                                         //最大转向角
    nh_.getParam("/mpc_controller/reference_velocity",v_ref);                                                //参考速度
    nh_.getParam("/mpc_controller/horizon",horizon_);                                                        //预测步长
    nh_.getParam("/mpc_controller/number_of_iteration",numOfIter);                                           //mpc迭代步长

    //矩阵初始化
    //这两个权重矩阵是预测空间下的状态量大小
    r_ = Matrix::Identity(basic_control_size,basic_control_size);                                   //控制权重矩阵
    q_ = Matrix::Identity(basic_state_size+basic_control_size,basic_state_size+basic_control_size); //状态权重矩阵
    q_ *= 20;
    state_mat_ = Matrix::Zero(basic_state_size,1);                                                  //误差状态矩阵
    control_mat_ = Matrix::Zero(basic_control_size,1);                                              //控制矩阵
    a_mat_ = Matrix::Zero(basic_state_size,basic_state_size);                                       //状态空间矩阵
    a_mat_d_ = Matrix::Zero(basic_state_size,basic_state_size);                                     //离散状态空间矩阵
    b_mat_ = Matrix::Zero(basic_state_size,1);                                                      //控制空间矩阵
    b_mat_d_ = Matrix::Zero(basic_state_size,1);                                                    //离散控制空间矩阵
    c_mat_ = Matrix::Zero(basic_state_size,1);                                                      //扰动空间矩阵
    c_mat_d_ = Matrix::Zero(basic_state_size,1);                                                    //离散扰动空间矩阵

    //计算整车质量
    double mass_front = mass_front_left_wheel + mass_front_right_wheel + mass_body / 2.0;           //前车质量
    double mass_rear = mass_rear_left_wheel + mass_rear_right_wheel + mass_body / 2.0;              //后车质量
    mass_ = mass_front + mass_rear;
    //计算前后轮轴距
    lf_ = wheel_base * (1 - mass_front / mass_);                                                    //前轮轴距
    lr_ = wheel_base * (1 - mass_rear / mass_);                                                     //后轮轴距
    //计算转动惯量
    izz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

    //最大转向度数到rad
    max_steer_angle *= M_PI/180.0;

    //获取路径名称
    string pkg_path = ros::package::getPath(pkg_name);
    waypoints_path = pkg_path+"/waypoints/"+waypoints_name;
    ROS_INFO("waypoints file is loaded at directory: %s",waypoints_path.c_str());
}

//加载路径点信息
void MpcController::loadPathInfo()
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
    }
    numOfWaypoints = (int)xr.size();    

}

//找到最近的路径点
int MpcController::findCloestIndex(const double x,const double y)
{
    vector<double> dist; 
    //遍历路径点计算与小车的距离
    for (int i = 0;i<numOfWaypoints;++i)
    {
        dist.push_back(pow(xr[i]-x,2)+pow(yr[i]-y,2));
    }
    //找到最小值的指针位置
    auto smallest = min_element(dist.begin(),dist.end());
    //求出从起始位置到最小点的下标
    int index = distance(dist.begin(),smallest);

    return index;
}

//误差计算模块
void MpcController::computeStateError(const double x,
                                        const double y, 
                                        const double v, 
                                        const double heading, 
                                        const double heading_rate, 
                                        const int index)
{
    double dx = x - xr[index];
    double dy = y - yr[index];

    //横向误差
    double ed = dy * cos(yawr[index]) - dx * sin(yawr[index]);
    //航向误差
    double e_psi = normalizeAngle(heading - yawr[index]);
    //横向误差率
    double ed_dot = v * sin(e_psi);
    //纵向速度
    double s_dot = v * cos(e_psi) / (1 - kappar[index] * ed);
    //航向误差率
    double e_psi_dot =  heading_rate - kappar[index] * s_dot;

    //把误差放入矩阵中
    state_mat_(0,0) = ed;
    state_mat_(1,0) = ed_dot;
    state_mat_(2,0) = e_psi;
    state_mat_(3,0) = e_psi_dot;
}

//计算状态空间矩阵
void MpcController::computeStateMatrix(const double linear_velocity)
{
    Matrix i_mat = Matrix::Identity(basic_state_size,basic_state_size);

    //状态矩阵
    /*
        [0,1,0,0;
        0,(-(c_f+c_r)/(m*v),(c_f+c_r)/m,-(c_f*l_f-c_r*l_r)/(m*v);
        0,0,0,1;
        0,(-(c_f*l_f-c_r*l_r))/(I_z*v),(c_f*l_f-c_r*l_r)/I_z,(-(c_f*l_f^2+c_r*l_r^2))/(I_z*v)];
    */
    // 4 x 4
    a_mat_(0,1) = 1.0;
    a_mat_(1,1) = -(cf_ + cr_) / (mass_ * linear_velocity);
    a_mat_(1,2) = (cf_ + cr_) / mass_;
    a_mat_(1,3) = -(cf_ * lf_ - cr_ * lr_) / (mass_ * linear_velocity);
    a_mat_(2,3) = 1.0;
    a_mat_(3,1) = -(cf_ * lf_ - cr_ * lr_) / (izz_ * linear_velocity);
    a_mat_(3,2) = (cf_ * lf_ - cr_ * lr_) / izz_;
    a_mat_(3,3) = -(cf_ * lf_ * lf_ + cr_ * lr_ * lr_) / (izz_ * linear_velocity);
    //离散化
    a_mat_d_ = (i_mat + 0.5 * ts_ * a_mat_) * (i_mat - 0.5 * ts_ * a_mat_).inverse();

    //控制矩阵
    /*
        [0;
        cf/m;
        0;
        (cf*lf)/Iz]
    */
    // 4 x 1
    b_mat_(1,0) = cf_ / mass_;
    b_mat_(3,0) = (cf_ * lf_) / izz_;
    //离散化处理
    b_mat_d_ = b_mat_ * ts_;

    //扰动矩阵
    /*
        [0;
        (cf*lf-cr*lr)/(m*v)-v;
        0;
        (lf*lf*cf+lr*lr*cr)/(izz*v)]
    */
    c_mat_(1,0) = (cf_ * lf_ - cr_ * lr_) / (mass_ * linear_velocity) - linear_velocity;
    c_mat_(3,0) = (cf_* lf_ * lf_ + cr_ * lr_ * lr_) / izz_ / linear_velocity;
    //离散化
    c_mat_d_ = c_mat_ * heading_error_rate * ts_;
}

//mpc求解器
double MpcController::mpcSolver()
{
    //设置某一时时刻下的转角限制
    // 1 x 1
    Matrix upper_bound = Matrix::Zero(basic_control_size,1);
    upper_bound << max_steer_angle;
    Matrix lower_bound = Matrix::Zero(basic_control_size,1);
    lower_bound << -max_steer_angle;
    //设置转角变化限制
    // 1 x 1
    Matrix upper_bound_delta = Matrix::Zero(basic_control_size,1);
    upper_bound_delta << 0.64;
    Matrix lower_bound_delta = Matrix::Zero(basic_control_size,1);
    lower_bound_delta << -0.64;

    // Y(t) = PSI * kesi + PHI * delta_U
    //构建预测空间
    //预测误差空间矩阵
    // 5 x 1
    Matrix kesi_mat = Matrix::Zero(basic_state_size+basic_control_size,1);
    kesi_mat.block(0,0,basic_state_size,1) = state_mat_;
    kesi_mat.block(4,0,basic_control_size,1) = control_mat_;

    //预测状态空间矩阵
    /*
        A = [a b]
            [O i]
    */
    // 5 x 5
    Matrix A_matrix_ =  Matrix::Zero(basic_state_size+basic_control_size,basic_state_size+basic_control_size);
    A_matrix_.block(0,0,basic_state_size,basic_state_size) = a_mat_d_;
    A_matrix_.block(0,4,b_mat_d_.rows(),b_mat_d_.cols()) = b_mat_d_;
    A_matrix_.block(4,4,1,1) = Matrix::Identity(1,1);

    //预测控制空间矩阵
    /*
        B = [b]
            [I]
    */
    // 
    // 5 x 1
    Matrix B_matrix_ = Matrix::Identity(b_mat_d_.rows()+basic_control_size,1);
    B_matrix_.block(0,0,b_mat_d_.rows(),1) = b_mat_d_;

    //预测扰动空间矩阵
    /*
        C = [c]
            [0]
    */
    Matrix C_matrix_ = Matrix::Zero(c_mat_d_.rows()+basic_control_size,1);
    C_matrix_.block(0,0,c_mat_d_.rows(),1) = c_mat_d_;

    //预测模型空间矩阵
    /*
        PSI = [A
                A^2
                A^3
                ...
                A^(Np)]
    */
    //先把A矩阵的各阶幂矩阵放进一个容器方便提取
    vector<Matrix> A_power_matrix(horizon_);
    A_power_matrix[0] = A_matrix_;
    for(int i = 1; i < horizon_; ++i)
    {
        A_power_matrix[i] = A_matrix_ * A_power_matrix[i-1];
    }
    // 150 x 5
    Matrix PSI_matrix = Matrix::Zero(A_matrix_.rows() * horizon_,A_matrix_.cols());
    for(int j = 0; j < horizon_; ++j)
    {
        
        PSI_matrix.block(j * A_matrix_.rows(),0,A_matrix_.rows(),A_matrix_.cols()) = A_power_matrix[j];
    }

    //预测控制空间矩阵
    /*
        PHI = [B 0 0 0 ... 0
                AB B 0 0 ... 0
                A^2*B AB B 0 ... 0]
                .  . . . ... .
                .  . . . ... .
                A^(Np)B ... ... B]
    */
    // 150 x 30
    Matrix PHI_matrix = Matrix::Zero(B_matrix_.rows() * horizon_ , B_matrix_.cols() * horizon_);
    PHI_matrix.block(0,0,B_matrix_.rows(),B_matrix_.cols()) = B_matrix_;
    for(int r = 1; r<horizon_; ++r)
    {
        for(int c = 0; c<r; ++c)
        {
             PHI_matrix.block(r * B_matrix_.rows(), c * B_matrix_.cols(), B_matrix_.rows(), B_matrix_.cols()) = A_power_matrix[r - c - 1] * B_matrix_; 
        }
        PHI_matrix.block(r * B_matrix_.rows(), r * B_matrix_.cols(), B_matrix_.rows(), B_matrix_.cols()) = B_matrix_;
    }
    //预测扰动空间矩阵
    /*
        GAMMA = [C
                AC+C
                A^2C+AC+C
                ...        
                SUM(A^iC)
                ]
    */
    Matrix GAMMA_matrix = Matrix::Zero(C_matrix_.rows() * horizon_, 1);
    GAMMA_matrix.block(0,0,C_matrix_.rows(),1) = C_matrix_;
    for(int r = 1; r < horizon_; ++r)
    {
        GAMMA_matrix.block(r*C_matrix_.rows(),0,C_matrix_.rows(),1) = A_power_matrix[r-1] * C_matrix_ 
                                                                    + GAMMA_matrix.block((r-1)*C_matrix_.rows(),0,C_matrix_.rows(),1);
    }

    //mpc预测问题转换成二次规划问题
    /*
        min J = 1/2 * delta_U.transpose() * H * delta_U + g.transpose() * delta_U
        H = PHI.transpose() * Qq * PHI + Rr
        G = PHI.trranspose() * Qq * E
        E = PSI * kesi
    */
    //设置E矩阵
    Matrix E_matrix = PSI_matrix * kesi_mat;            // 150 x 1
    //设置Qq矩阵
    Matrix Qq_matrix = Matrix::Identity(PHI_matrix.rows(),PHI_matrix.rows());       // 150 x 150
    //设置Rr矩阵
    Matrix Rr_matrix = Matrix::Identity(PHI_matrix.cols(),PHI_matrix.cols());       // 30 x 30
    for (int i = 0;i < horizon_; ++i)
    {
        Qq_matrix.block(i*q_.rows(),i*q_.rows(),q_.rows(),q_.rows()) = q_;
        Rr_matrix.block(i*r_.rows(),i*r_.rows(),r_.cols(),r_.cols()) = r_;
    }
    //设置H矩阵
    Matrix H_matrix = PHI_matrix.transpose() * Qq_matrix * PHI_matrix + Rr_matrix;  // 30 x 30
    //设置G矩阵
    Matrix G_matrix = PHI_matrix.transpose() * Qq_matrix * (E_matrix+GAMMA_matrix);                // 30 x 1

    //设置二次规划上下限
    /*
        Umin - Ut <= Ai * delta_U <= Umax - Ut      -- 约束
        delta_U_min <= delta_U <= delta_U_max       -- 界限
    */
    //delta_U_min 和 delta_U_max 上下界
    Matrix ll_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);      // 30 x 1
    Matrix uu_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);      // 30 x 1
    //不等式上下界
    Matrix Ut_matrix = Matrix::Zero(horizon_* control_mat_.rows(),1);       // 30 x 1
    Matrix Umax_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);    // 30 x 1
    Matrix Umin_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);    // 30 x 1
    for(int j = 0 ; j < horizon_ ; ++j)
    {
        ll_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = lower_bound_delta;
        uu_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = upper_bound_delta;
        Ut_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = control_mat_;
        Umax_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = upper_bound;
        Umin_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = lower_bound;
    }
    //设置不等式右边的边界矩阵
    Matrix inequality_lower_boundary_matrix = Matrix::Zero(Umax_matrix.rows(),Umax_matrix.cols());      // 30 x 1
    inequality_lower_boundary_matrix = Umin_matrix - Ut_matrix;
    Matrix inequality_upper_boundary_matrix = Matrix::Zero(Umin_matrix.rows(),Umin_matrix.cols());      // 30 x 1
    inequality_upper_boundary_matrix = Umax_matrix - Ut_matrix;
    //设置不等式左边的边界矩阵
    Matrix inequality_constraint_matrix = Matrix::Identity(inequality_lower_boundary_matrix.rows(),inequality_lower_boundary_matrix.rows());    // 30 x 30
    for (int r = 1; r < inequality_lower_boundary_matrix.rows(); ++r)
    {
        for (int c = 0; c < r; ++c)
        {
            inequality_constraint_matrix(r,c) = 1;
        }
    }
    //设置qp解算器
    //主要是要把矩阵信息转换成qp解算器可以读取的信息类型
    double delta_steer = solveQP(H_matrix, G_matrix, 
                            inequality_constraint_matrix, inequality_lower_boundary_matrix,
                            inequality_upper_boundary_matrix , ll_matrix, uu_matrix, numOfIter);
    
    control_mat_(0,0) = kesi_mat(4,0) + delta_steer;

    return control_mat_(0,0);

}

//qp解算器
//返回控制量
double MpcController::solveQP(const Matrix& H_matrix,
                              const Matrix& G_matrix, 
                              const Matrix& inequality_matrix, 
                              const Matrix& inequality_lower_boundary_matrix,
                              const Matrix& inequality_upper_boundary_matrix,
                              const Matrix& lower_bound, 
                              const Matrix& upper_bound, 
                              const int numOfIter)
{
    //变量数
    const qpOASES::int_t num_param_ = H_matrix.rows();       //就是整个控制量误差空间的数目
    const qpOASES::int_t num_constraints_ = inequality_matrix.rows();
    //创建qp解算器对象
    qpOASES::QProblem mpc_qp(num_param_,num_constraints_);

    //把H矩阵和G矩阵元素放进数组中
    //G矩阵的行数与H矩阵是一致的
    const int numOfHmatrixElements = H_matrix.rows() * H_matrix.cols();
    //double h_matrix[numOfHmatrixElements];
    qpOASES::real_t h_matrix[numOfHmatrixElements];
    
    const int numOfGmatrixElements = G_matrix.rows();
    //double g_matrix[numOfGmatrixElements];
    qpOASES::real_t g_matrix[numOfGmatrixElements];
    
    int index = 0;
    //循环遍历并放入数组中
    for (int r = 0; r < H_matrix.rows();++r)
    {
        //G矩阵的行数与H矩阵是一致的
        g_matrix[r] = G_matrix(r,0);
        for (int c = 0; c < H_matrix.cols(); ++c)
        {
            h_matrix[index++] = H_matrix(r,c);
        }
    }

    //上下限
    qpOASES::real_t lb[num_param_];
    qpOASES::real_t ub[num_param_];
    for (int i = 0; i < num_param_ ; ++i)
    {
        lb[i] = lower_bound(i,0);
        ub[i] = upper_bound(i,0);
    }

    //约束条件
    qpOASES::real_t inequality_constraint[num_param_ * num_constraints_];
    qpOASES::real_t lbA[num_constraints_];
    qpOASES::real_t ubA[num_constraints_];
    index = 0;
    for (int r = 0; r < inequality_matrix.rows(); ++r)
    {
        //约束条件上下界
        lbA[r] = inequality_lower_boundary_matrix(r,0);
        ubA[r] = inequality_upper_boundary_matrix(r,0);

        for (int c = 0; c < inequality_matrix.cols(); ++c)
        {
            inequality_constraint[index++] = inequality_matrix(r,c);
        }
    }
    //qp解算
    qpOASES::int_t iterNum = 50;
    auto ret = mpc_qp.init(h_matrix,g_matrix,inequality_constraint,lb,ub,lbA,ubA,iterNum);
    if (ret != qpOASES::SUCCESSFUL_RETURN)
    {
        if (ret == qpOASES::RET_MAX_NWSR_REACHED)
        {
            ROS_INFO("qpOASES solver failed due to reached max iteration");
        }
        else
        {
            ROS_INFO("qpOASES solver filaed due to some reasons...Please check...");
        }
    }

    //返回结果是delta_U的整个预测空间
    double result[num_param_];
    mpc_qp.getPrimalSolution(result);

    return result[0];
}

//前馈补偿
//与LQR不同，由于整体控制不在为 u =-kx+delta_f,所以原来的前馈补偿含有LQR的项被直接舍去（这是Apollo的看法）
double MpcController::feedForwardControl(const double v, const double kappa)
{
    double kv = lr_ * mass_/ cf_ / wheel_base - lf_ * mass_ / cr_ / wheel_base;
    double steering_angle_feedforward = wheel_base * kappa + kv * v * v * kappa ;

    return steering_angle_feedforward;
}

//计算最终转向角
double MpcController::computeSteering(const double x,
                                        const double y,
                                        const double v,
                                        const double heading, 
                                        const double heading_rate)
{
    //找到最近路径点下标
    int index = findCloestIndex(x,y);
    //计算误差
    computeStateError(x,y,v,heading,heading_rate,index);
    //计算状态空间和控制空间矩阵
    computeStateMatrix(v);
    //前馈补偿
    double steer_feedforawrd = feedForwardControl(v,kappar[index]);
    //mpc控制
    double steer = mpcSolver();
    return steer+steer_feedforawrd;
}

//发布全局路径
void MpcController::pubPath()
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
double MpcController::normalizeAngle(const double angle)
{
    //对角度取模
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if(a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;   
}

//odom里程计回调函数
void MpcController::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msgs)
{
    //发布路径
    pubPath();

    //获取当前车辆位置信息
    double x = odom_msgs->pose.pose.position.x;     //x
    double y = odom_msgs->pose.pose.position.y;     //y
    //用tf2设置四元数
    tf2::Quaternion q(
        odom_msgs->pose.pose.orientation.x,
        odom_msgs->pose.pose.orientation.y,
        odom_msgs->pose.pose.orientation.z,
        odom_msgs->pose.pose.orientation.w
        );
    //把四元数转换成欧拉角
    double roll,pitch,yaw;
    tf2::Matrix3x3 m_rpy(q);
    m_rpy.getRPY(roll,pitch,yaw);
    //获取速度信息
    double vx = odom_msgs->twist.twist.linear.x;
    double vy = odom_msgs->twist.twist.linear.y;
    double linear_velocity = sqrt(vx * vx + vy * vy);           //车辆线速度
    double heading_rate = odom_msgs->twist.twist.angular.z;     //车辆航向速度
    double steering = computeSteering(x,y,linear_velocity,yaw,heading_rate);

    //发布动作信息
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = v_ref;
    cmd_vel.angular.z = steering;
    pub_cmd.publish(cmd_vel);

    ROS_INFO("Velocity = %f, Steering angle = %f",cmd_vel.linear.x,(cmd_vel.angular.z)*180.0/M_PI);
}

//主程序接口
int main(int argc,char **argv)
{
    //初始化ros
    ros::init(argc,argv,"mpc_controller");

    //创建mpc控制器对象
    MpcController mpccontroller;

    //启动节点
    ros::spin();

    //关闭节点
    ros::shutdown();

    return 0;

}