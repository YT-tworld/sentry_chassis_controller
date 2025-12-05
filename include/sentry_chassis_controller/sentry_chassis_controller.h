#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

// 速度接口核心头文件（新手友好）
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h> // 速度接口
#include <pluginlib/class_list_macros.h>  // 注册插件必须
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>  // ROS官方PID类（必须加）
#include <dynamic_reconfigure/server.h>  // 动态调参服务器
// 自动生成的动态调参配置头文件（名字要和你的cfg文件一致）
#include <sentry_chassis_controller/ChassisPIDConfig.h>
#include <algorithm>
#include <nav_msgs/Odometry.h>//里程计消息文件
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath> //sin,cos
#include <tf2_ros/buffer.h>   //提供tf2_ros::Buffer类的定义；
#include <tf2_ros/transform_listener.h> //tf2_ros::TransformListener类的定义
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>   //欧拉角转换所需

namespace sentry_chassis_controller {

// 控制器类：继承速度接口（VelocityJointInterface）
class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  /**
   * @brief 控制器初始化（仅返回成功，无复杂逻辑）
   * @param hw 硬件接口指针（速度接口）
   *        *这是 C++ 多态的核心用法：父类指针指向子类对象，通过父类指针hw调用子类的实现
   *         rm_gazebo → gazebo_ros_control → controller_manager(gazebo_ros_control 创建硬件接口实现类，并注册到 controller_manager)
   * @param nh 节点句柄(传过来的nh已经被controller_manager指定了命名空间就是你launch中args的控制器名字)，ros_master（用于当前控制器下的读取参数/订阅话题）
   * @return 初始化成功返回true，失败返回false
   */
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

  /**
   * @brief 控制周期回调，默认频率100HZ，可在controller_manager中设置
   * @param time  当前 update 函数被调用时的系统绝对时间（从 ROS 节点启动时开始计时，单位是秒）
   * @param period 上一次调用 update 函数到本次调用的时间间隔（单位是秒）
   */
  void update(const ros::Time& time, const ros::Duration& period) override;//override是提示你，这个是重写父类的虚函数
  
  /**
   * @brief 动态参数服务器的回调函数
   * @param  config 从客户端传来的参数
   * @param  level 只有 “参数全重置” 时 level == 0
   */
  //void dynParamCallback(sentry_chassis_controller::ChassisPIDConfig& config, uint32_t level) ;

  void dynParamRead();

private:
  // 1. 关节句柄（存储8个关节的控制接口）
  std::vector<hardware_interface::JointHandle> joint_handles_;
  // 2. 8个PID控制器（对应8个关节）
  std::vector<control_toolbox::Pid> pid_controllers_;
  // 3. 电机目标速度，舵机目标角度
  std::vector<double> target_motor;
  std::vector<double> target_servo;
  // 4. 动态调参服务器（支持rqt_reconfigure）
  std::shared_ptr<dynamic_reconfigure::Server<sentry_chassis_controller::ChassisPIDConfig>> dyn_server;
  //dynamic_reconfigure::Server<sentry_chassis_controller::ChassisPIDConfig>::CallbackType dyn_cbType;//回调函数
  
  // 5.逆运动学
  // 逆运动学相关成员
  ros::Subscriber cmd_vel_sub_;  // 订阅/cmd_vel话题（接收底盘速度指令）
  double wheel_radius_;          // 轮子半径（m，从配置文件读取）
  double wheelbase_;             // 轴距（前后轮距离，m）
  double track_width_;           // 轮距（左右轮距离，m）
  double max_steer_angle_;       // 舵机最大转向角（rad，默认±90度=1.57rad）
  // 逆运动学核心函数
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);  // /cmd_vel回调（解算转向角和转速）
  template<typename T> T clamp(T val, T min_val, T max_val);  // 工具函数：限制值范围

  // 6.里程计与tf坐标变换
  // 新增：里程计相关成员
  ros::Publisher odom_pub_;                  // 发布odom话题的Publisher
  tf2_ros::TransformBroadcaster tf_broadcaster_; // 广播坐标变换的对象
  nav_msgs::Odometry odom_msg_;              // 里程计消息载体
  tf2::Quaternion odom_quat_;                // 存储旋转四元数（将z轴角速度转为姿态）
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_th_ = 0.0; // 里程计坐标（x/y/航向角）
  ros::Time last_odom_time_;                 // 上一次更新里程计的时间（计算位移用）
  // 里程计核心函数声明
  void initOdom(ros::NodeHandle& nh);        // 初始化里程计（创建Publisher等）
  void updateOdom(const ros::Time& time, const ros::Duration& period); // 周期更新里程计

  // 7.odom里程计坐标系的逆运动实现
  // 新增：TF监听相关（用于获取odom→base_link的变换）
  tf2_ros::Buffer tf_buffer_;                  // TF数据缓存器（存储历史变换）
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF监听器（订阅TF变换）
  bool is_global_vel_mode_;                    // 速度模式：true=全局坐标系，false=底盘坐标系
  double yaw_from_tf_;                         // 从TF获取的机器人航向角（odom→base_link的yaw）

  // 新增：声明速度转换函数
  bool transformWorldVelToBaseVel(const geometry_msgs::Twist::ConstPtr& world_vel, 
                                  geometry_msgs::Twist& base_vel);
  // 新增：从配置文件读取速度模式
  void initVelMode(ros::NodeHandle& nh);

};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

