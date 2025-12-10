#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>

// 全局变量声明
struct termios old_tio; //终端原始属性（用于退出时恢复）
ros::Publisher cmd_vel_pub;
double max_linear_speed = 0.5;   // 最大线速度(m/s)
double max_angular_speed = 1.57; // 最大角速度(rad/s，约90度/秒)
bool terminal_inited = false;    // 终端初始化标志

// 信号处理函数：恢复终端属性并退出
void sig_handler(int sig) {
    if (terminal_inited) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // 恢复终端属性
    }
    ros::shutdown();
    std::cout << "\n程序已退出，终端属性已恢复" << std::endl;
    exit(0);
}

// 采用Linux 的终端 I/O 控制接口（termios）修改终端属性（替代了ncurses）
// 初始化终端为非阻塞、无缓冲、无回显模式
bool init_terminal() {
    struct termios new_tio;
    // 获取终端原始属性
    if (tcgetattr(STDIN_FILENO, &old_tio) == -1) {
        perror("tcgetattr 失败");
        return false;
    }
    new_tio = old_tio;
    // 关闭行缓冲（ICANON）、关闭回显（ECHO）
    new_tio.c_lflag &= ~(ICANON | ECHO);
    // 设置最小读取字符数为0（非阻塞）
    new_tio.c_cc[VMIN] = 0;
    // 设置读取超时时间为0（立即返回）
    new_tio.c_cc[VTIME] = 8;//800ms
    // 应用新的终端属性
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) == -1) {
        perror("tcsetattr 失败");
        return false;
    }
    // 设置标准输入为非阻塞模式
    if (fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        perror("fcntl 失败");
        return false;
    }
    terminal_inited = true;
    return true;
}


// 处理键盘输入并更新速度指令
void handle_keyboard_input(geometry_msgs::Twist& twist) {
    char key[16];//最多读16个字符
    // 非阻塞读取键盘输入
    ssize_t read_len = read(STDIN_FILENO, key, sizeof(key));//chae是1字节，sizeof()得到的是字节长度
    // 初始化按键状态标志
    bool has_w = false, has_s = false, has_a = false, has_d = false, has_q = false, has_e = false,has_out=false;

    // 仅处理实际读取到的字符
    if (read_len > 0) {
        //重置速度
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        // 遍历实际读取的字符数（而非数组总长度）
        for (int i = 0; i < read_len; i++) {
            switch (key[i]) {
                case 'w': case 'W': has_w = true; break;
                case 's': case 'S': has_s = true; break;
                case 'a': case 'A': has_a = true; break;
                case 'd': case 'D': has_d = true; break;
                case 'q': case 'Q': has_q = true; break;
                case 'e': case 'E': has_e = true; break;
                case '\x03': has_out = true; break;// Ctrl+C（ASCII码3）
                default: break;
            }
        }
        // 清空数组（避免残留数据影响下次读取）
        memset(key, 0, sizeof(key));
    }

    // 根据按键状态设置速度（避免累加，仅取存在性）
    if (has_w) twist.linear.x += max_linear_speed;
    if (has_s) twist.linear.x -= max_linear_speed;
    if (has_a) twist.linear.y += max_linear_speed;
    if (has_d) twist.linear.y -= max_linear_speed;
    if (has_q) twist.angular.z += max_angular_speed;
    if (has_e) twist.angular.z -= max_angular_speed;
    if (has_out) sig_handler(SIGINT);
}




// // 处理键盘输入并更新速度指令
// void handle_keyboard_input(geometry_msgs::Twist& twist) {
//     char key;
//     // 非阻塞读取键盘输入
//     ssize_t read_len = read(STDIN_FILENO, &key, 1);
//     if (read_len > 0) {
//         // 重置速度（默认停止）
//         twist.linear.x = 0.0;
//         twist.linear.y = 0.0;
//         twist.angular.z = 0.0;
//         // 处理按键
//         switch (key) {
//             case 'w': case 'W':
//                 twist.linear.x = max_linear_speed; // 前进
//                 break;
//             case 's': case 'S':
//                 twist.linear.x = -max_linear_speed; // 后退
//                 break;
//             case 'a': case 'A':
//                 twist.linear.y = max_linear_speed; // 左移
//                 break;
//             case 'd': case 'D':
//                 twist.linear.y = -max_linear_speed; // 右移
//                 break;
//             case 'q': case 'Q':
//                 twist.angular.z = max_angular_speed; // 逆时针旋转（左转）
//                 break;
//             case 'e': case 'E':
//                 twist.angular.z = -max_angular_speed; // 顺时针旋转（右转）
//                 break;
//             case ' ':
//                 // 空格停止（速度保持0）
//                 break;
//             case '\x03': // Ctrl+C（ASCII码3）
//                 sig_handler(SIGINT);
//                 break;
//             default:
//                 // 未知按键，保持停止
//                 break;
//         }
//     }
// }

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "node_mykeyboard");
    ros::NodeHandle nh("/mykeyboard"); // 私有命名空间，便于读取参数

    // 从参数服务器读取配置参数（默认值为原固定值）
    nh.param<double>("max_linear_speed", max_linear_speed, 0.5);
    nh.param<double>("max_angular_speed", max_angular_speed, 1.57);
    std::string cmd_vel_topic;
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");

    // 初始化发布者
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);

    // 注册信号处理函数（Ctrl+C）
    signal(SIGINT, sig_handler);

    // 初始化终端
    if (!init_terminal()) {
        ROS_ERROR("终端初始化失败！");
        return 1;
    }

    // 打印启动信息
    std::cout << "键盘操控节点已启动！" << std::endl;
    std::cout << "控制说明：" << std::endl;
    std::cout << "W:前进  S:后退  A:左移  D:右移" << std::endl;
    std::cout << "Q:逆时针旋转  E:顺时针旋转  空格:停止" << std::endl;
    std::cout << "最大线速度：" << max_linear_speed << " m/s" << std::endl;
    std::cout << "最大角速度：" << max_angular_speed << " rad/s" << std::endl;
    std::cout << "按Ctrl+C退出" << std::endl;

    // 速度指令消息初始化
    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;

    // 主循环
    ros::Rate loop_rate(50); // 50Hz循环频率（发布读取频率）
    while (ros::ok()) {
        // 处理键盘输入
        handle_keyboard_input(twist);
        // 发布速度指令
        cmd_vel_pub.publish(twist);
        // 循环频率控制
        loop_rate.sleep();
        // 处理ROS回调（若有）
        ros::spinOnce();
    }

    // 恢复终端属性（冗余保护）
    if (terminal_inited) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    }
    return 0;
}



























// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <ncurses.h>
// #include <signal.h>
// #include <iostream>

// // 全局变量：速度参数、发布者、节点句柄
// ros::Publisher cmd_vel_pub;
// double linear_x = 0.0, linear_y = 0.0, angular_z = 0.0;
// double max_linear = 1.0;    // 最大线速度 (m/s)
// double max_angular = 1.57;  // 最大角速度 (rad/s，约90度/秒)
// double speed_step = 0.1;    // 速度调节步长

// // 信号处理函数：优雅关闭ncurses和ROS节点
// void sig_handler(int sig) {
//     endwin();  // 关闭ncurses窗口
//     ros::shutdown();
//     std::cout << "键盘操控节点已退出" << std::endl;
//     exit(0);
// }

// // 初始化ncurses
// void init_ncurses() {
//     initscr();          // 初始化ncurses
//     cbreak();           // 禁用行缓冲，字符直接输入
//     noecho();           // 不回显输入字符
//     keypad(stdscr, TRUE); // 启用功能键
//     nodelay(stdscr, TRUE); // 非阻塞输入
//     curs_set(0);        // 隐藏光标
// }

// // 键盘输入处理函数
// void handle_key_input() {
//     int ch;
//     while (ros::ok()) {
//         ch = getch();
//         // 重置速度（每次按键后重置，实现点动控制；若需持续运动，可移除重置逻辑）
//         linear_x = 0.0;
//         linear_y = 0.0;
//         angular_z = 0.0;

//         switch (ch) {
//             case 'w': // 前进（x轴正方向）
//                 linear_x = max_linear;
//                 break;
//             case 's': // 后退（x轴负方向）
//                 linear_x = -max_linear;
//                 break;
//             case 'a': // 左移（y轴正方向，或左转，根据机器人类型调整）
//                 // 若为全向轮机器人：左移（y轴正方向）
//                 linear_y = max_linear;
//                 // 若为差分驱动机器人：左转（z轴正方向）
//                 // angular_z = max_angular;
//                 break;
//             case 'd': // 右移（y轴负方向，或右转，根据机器人类型调整）
//                 // 若为全向轮机器人：右移（y轴负方向）
//                 linear_y = -max_linear;
//                 // 若为差分驱动机器人：右转（z轴负方向）
//                 // angular_z = -max_angular;
//                 break;
//             case 'q': // 顺时针旋转（z轴负方向）
//                 angular_z = -max_angular;
//                 break;
//             case 'e': // 逆时针旋转（z轴正方向）
//                 angular_z = max_angular;
//                 break;
//             case ' ': // 空格：停止所有运动
//                 linear_x = 0.0;
//                 linear_y = 0.0;
//                 angular_z = 0.0;
//                 break;
//             case KEY_RESIZE: // 窗口大小调整
//                 refresh();
//                 break;
//             default:
//                 break;
//         }

//         // 构造Twist消息并发布
//         geometry_msgs::Twist twist;
//         twist.linear.x = linear_x;
//         twist.linear.y = linear_y;
//         twist.linear.z = 0.0;
//         twist.angular.x = 0.0;
//         twist.angular.y = 0.0;
//         twist.angular.z = angular_z;
//         cmd_vel_pub.publish(twist);

//         // 短暂休眠，降低CPU占用
//         usleep(50000); // 50ms
//     }
// }

// int main(int argc, char** argv) {
//     // 初始化ROS节点
//     ros::init(argc, argv, "keyboard_teleop");
//     ros::NodeHandle nh;

//     // 读取参数（可选：从参数服务器配置速度限制和步长）
//     nh.param<double>("max_linear", max_linear, 1.0);
//     nh.param<double>("max_angular", max_angular, 1.57);
//     nh.param<double>("speed_step", speed_step, 0.1);

//     // 发布/cmd_vel话题（可根据实际需求修改话题名）
//     cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

//     // 注册信号处理函数（处理Ctrl+C退出）
//     signal(SIGINT, sig_handler);

//     // 初始化ncurses并处理键盘输入
//     try {
//         init_ncurses();
//         printw("键盘操控节点已启动！\n");
//         printw("控制说明：\n");
//         printw("W: 前进   S: 后退   A: 左移   D: 右移\n");
//         printw("Q: 顺时针旋转   E: 逆时针旋转   空格: 停止\n");
//         printw("按Ctrl+C或Q键退出\n");
//         refresh();
//         handle_key_input();
//     } catch (...) {
//         endwin();
//         ROS_ERROR("ncurses初始化失败！");
//         return 1;
//     }

//     // 关闭ncurses
//     endwin();
//     return 0;
// }