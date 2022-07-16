#ifndef KEY_CONTROL_H
#define KEY_CONTROL_H

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class KeyboardCtrl
{
    struct termios initial_settings, new_settings;
    int peek_character = -1;

    public:
        //构造函数：初始化键盘监听器的设定
        KeyboardCtrl();
        //析构函数：关闭键盘监听器
        ~KeyboardCtrl();
        //获取键盘按键
        int get_press_key();

    private:
        //初始化键盘设定
        void init_keyboard();

        //关闭键盘监听器
        void close_keyboard();

        int kbhit();
        
        //返回按键值
        int readch();


};
#endif  // KEY_CONTROL_H