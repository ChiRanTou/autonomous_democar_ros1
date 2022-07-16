#include <keyboard_control.h>

//构造函数：初始化键盘监听器的设定
KeyboardCtrl::KeyboardCtrl()
{
    init_keyboard();
}

//析构函数：关闭键盘监听器
KeyboardCtrl::~KeyboardCtrl()
{
    close_keyboard();
}

//获取键盘按键
int KeyboardCtrl::get_press_key()
{
    kbhit();
    return readch();
}

//初始化键盘设定
void KeyboardCtrl::init_keyboard()
{
    /*
        c_ciflag: 输入模式标志，控制终端输入方式
        c_oflag: 输出模式标志，控制终端输出方式
        c_cflag: 控制模式标志，指定终端硬件控制信息
        c_lflag: 本地模式标志，控制终端编辑功能
        c_cc: 控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
    */
    //(文件描述符，设定)
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    //使用标准输入模式 | 显示输入字符
    new_settings.c_lflag &= ~(ICANON | ECHO);
    //VEOL: 附加的end of life字符
    new_settings.c_cc[VEOL] = 1;
    //VEOF: end of life字符
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);

}

//关闭键盘监听器
void KeyboardCtrl::close_keyboard()
{   
    //直接设置为默认
    tcsetattr(0, TCSANOW, &initial_settings);
}

int KeyboardCtrl::kbhit()
{
    unsigned char ch;
    int nread;

    if(peek_character != -1) {return 1;}
    //VMIN: 非规范模式读取时的最小字符数 
    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    if (nread == 1)
    {
        peek_character = ch;
        return 1;
    }
    return 0;

}

//返回按键值
int KeyboardCtrl::readch()
{
    char ch;
    if (peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}