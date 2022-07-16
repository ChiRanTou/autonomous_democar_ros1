#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;

class WaypointLoader
{
    public:
        //创建一个路径加载器，赋上文件路径名
        WaypointLoader(const string name);
        //析构函数
        ~WaypointLoader(){};
        //加载路径并返回是否读取成功的信息
        bool load_waypoints();
        //获取路径点
        //0：x点
        //1：y点
        //2：参考航向角yawr
        //3：参考路径曲率
        vector<vector<double>> get_waypoints();

    private:
        //路径文件名
        string path_name;
        //路径信息
        vector<vector<double>> wp;
};

