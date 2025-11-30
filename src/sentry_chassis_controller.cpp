#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.h>  // 注册插件必须

namespace sentry_chassis_controller {

//1.初始化控制器
bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) 
{ 
  setlocale(LC_ALL,"");//避免日志中中文乱码 

  // 1. 从配置文件读取关节名
  std::vector<std::string> joint_names;
  if (!nh.getParam("joint_names", joint_names) || joint_names.size() != 8) {
    ROS_ERROR("请在配置文件中设置8个关节名!");
    return false;
  }

  // 2. 获取8个关节的控制句柄
  for (const auto& name : joint_names) {
    joint_handles_.push_back(hw->getHandle(name));
  }

  // 3. 初始化8个PID控制器（从配置文件加载参数）
  pid_controllers_.resize(8);
  for (int i = 0; i < 8; ++i) {
    std::string pid_param = "pid/" + joint_names[i];
    if (!pid_controllers_[i].init(ros::NodeHandle(nh, pid_param))) {
      ROS_ERROR("加载关节[%s]的PID参数失败!", joint_names[i].c_str());
      return false;
    }
  }

  // 4-1. 初始化动态调参服务器（支持rqt_reconfigure），要在堆上new否则，init后服务器就被销毁了
  dyn_server.reset(new dynamic_reconfigure::Server<sentry_chassis_controller::ChassisPIDConfig>(nh));
  // // 4-2. 绑定回调函数：把 dynParamCallback 绑定到服务器
  // // 用 std::bind 把成员函数和 this 指针绑定（关键！让服务器能找到这个函数）
  // auto dyn_cbType = std::bind(&SentryChassisController::dynParamCallback, 
  //                         this,  // 必须传 this 指针，告诉函数是哪个类的实例
  //                         std::placeholders::_1,  // 占位符：对应 config 参数
  //                         std::placeholders::_2); // 占位符：对应 level 参数
  // // 4-3. 给服务器设置回调函数                        
  // dyn_server->setCallback(dyn_cbType);


  // 5. 初始化期望速度（默认0）
  target_motor.resize(4, 20.0);
  target_servo.resize(4, 0.0);
  ROS_INFO("控制器初始化成功，加载了8个关节的PID！");
  return true;  // 必须返回true，否则控制器加载失败
}

//2.更新
void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
  static int count = 0;
  const int print_interval = 100;  // 100Hz控制频率 → 每100帧（1秒）打印一次
  if (++count % print_interval == 0) {
     ROS_INFO("controller is running now!!!!!!!");
  }
    //读取动态参数
    this->dynParamRead();
    // 遍历8个关节，逐个执行PID控制
    for (int i = 0; i < 4; ++i) {
    // 1. 读取关节当前电机角速度，舵机以一个固定点为起始转过角度
    double current_vel = joint_handles_[i].getVelocity();
    double current_position =joint_handles_[i+4].getPosition();
    // 2. 计算速度误差（期望速度 - 当前速度）
    double error_v = target_motor[i] - current_vel;
    double error_p = target_servo[i] - current_position;
    // 3. 执行PID计算，得到控制量
    double pid_v = pid_controllers_[i].computeCommand(error_v, period);
    double pid_p = pid_controllers_[i+4].computeCommand(error_p, period);
    // 4. 将控制量PID发送给关节
    joint_handles_[i].setCommand(pid_v);
    joint_handles_[i+4].setCommand(pid_p);
  }
}

//  // 3.动态参数服务器的回调函数
// void SentryChassisController::dynParamCallback(sentry_chassis_controller::ChassisPIDConfig& config, uint32_t level)
// {
//   for (int i = 0; i < 4; ++i) 
//   {
//     this->pid_controllers_[i].initPid(config.Kp_motor, config.Ki_motor, config.Kd_motor  , config.i_clamp_motor,-config.i_clamp_motor);
//     this->pid_controllers_[i+4].initPid(config.Kp_servo, config.Ki_servo, config.Kd_servo, config.i_clamp_servo,-config.i_clamp_servo);
//     this->target_motor[i]=config.target_motor_all;//更新所有电机目标速度
//     this->target_servo[i]=config.target_servo_all;//更新所有舵机目标角度
//   }

// }

// 2. 核心：参数读取函数（读取参数服务器的参数，更新PID和目标值）
void SentryChassisController::dynParamRead() {
  // 定义临时变量存储读取的参数（避免直接修改本地变量导致的中间状态错误）
  double Kp_motor, Ki_motor, Kd_motor, i_clamp_motor;
  double Kp_servo, Ki_servo, Kd_servo, i_clamp_servo;
  double target_motor_all, target_servo_all;

  // 从参数服务器读取参数（参数名要和 .cfg 完全一致！）
  // 注意：如果控制器有命名空间，需在参数名前加命名空间（比如 "sentry_controller/Kp_motor"）
  ros::NodeHandle nh("/controller/sentry_chassis_controller"); // 若有命名空间，改为 ros::NodeHandle nh("sentry_controller");

  // 读取电机组参数（.cfg中定义的参数名）
  nh.param<double>("Kp_motor", Kp_motor, 1.0);          // 第二个参数是默认值（参数不存在时用）
  nh.param<double>("Ki_motor", Ki_motor, 0.1);
  nh.param<double>("Kd_motor", Kd_motor, 0.05);
  nh.param<double>("i_clamp_motor", i_clamp_motor, 0.5);
  // 读取舵机组参数
  nh.param<double>("Kp_servo", Kp_servo, 5.0);
  nh.param<double>("Ki_servo", Ki_servo, 0.2);
  nh.param<double>("Kd_servo", Kd_servo, 0.1);
  nh.param<double>("i_clamp_servo", i_clamp_servo, 1.0);
  // 读取目标值参数
  nh.param<double>("target_motor_all", target_motor_all, 0.0);
  nh.param<double>("target_servo_all", target_servo_all, 0.0);

  // 更新电机组PID参数（4个电机共用一套参数）,舵机组PID参数（4个舵机共用一套参数）
  for (int i = 0; i < 4; ++i) {
    pid_controllers_[i].initPid(Kp_motor, Ki_motor, Kd_motor, i_clamp_motor, -i_clamp_motor);
    target_motor[i] = target_motor_all;
    pid_controllers_[i+4].initPid(Kp_servo, Ki_servo, Kd_servo, i_clamp_servo, -i_clamp_servo);
    target_servo[i] = target_servo_all;
  }
  // 可选：打印参数更新日志（每秒打印一次，避免刷屏）
  static ros::Time last_print = ros::Time::now();
  if ((ros::Time::now() - last_print).toSec() > 1.0) {
    ROS_INFO("参数更新：电机P=%.2f，舵机P=%.2f，电机目标速度=%.2f",
             Kp_motor, Kp_servo, target_motor_all);
    last_print = ros::Time::now();
  }
}

}  // namespace sentry_chassis_controller

// 注册控制器为ROS插件（关键！让controller_manager能找到这个控制器）
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
