#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

// 速度接口核心头文件（新手友好）
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h> // 速度接口
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>  // ROS官方PID类（必须加）
#include <dynamic_reconfigure/server.h>  // 动态调参服务器
// 自动生成的动态调参配置头文件（名字要和你的cfg文件一致）
#include <sentry_chassis_controller/ChassisPIDConfig.h>

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
  
};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

