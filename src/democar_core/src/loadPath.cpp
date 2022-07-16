#include "loadPath.h"
//构造函数
//从外部获取路径名称
WaypointLoader::WaypointLoader(const string name)
{
    //设置文件名
    path_name = name;

}

//加载路径点
bool WaypointLoader::load_waypoints()
{
    bool fileLoadFlag;
    ifstream ifs;

    ifs.open(path_name,ios::in);

    //判断是否成功打开文件
    if(!ifs.is_open())
    {
        fileLoadFlag = false;
    }

    //判断文件内部是否为空
    char ch;
    ifs >> ch;
    if(ifs.eof())
    {
        fileLoadFlag = false;
    }
    ifs.putback(ch);

    //读取文件中的内容
    string line;
    string field;

    //getline(ifs,line)表示把每一行读取CSV文件中的数据
    while (getline(ifs,line))
    {
        vector<double> v;
        //第一列x载入
        istringstream sin(line);        //将整行字符串line读入到sin字符串流中
        getline(sin,field,',');         //将sin字符串流中的值读取到field中，用逗号分隔
        v.push_back(atof(field.c_str()));

        //第二列y载入
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));

        //第三列yaw载入
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));

        //第四列kappa载入
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));
        //把位置信息全部装入wp容器中
        wp.push_back(v);
        
    }
    fileLoadFlag = true;
    //关闭文件
    ifs.close();

    return fileLoadFlag;
}

//获取路径点
vector<vector<double>> WaypointLoader::get_waypoints()
{
    return wp;
}