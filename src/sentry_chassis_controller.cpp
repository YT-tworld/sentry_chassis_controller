#include "sentry_chassis_controller/sentry_chassis_controller.h"


namespace sentry_chassis_controller {

//【一.初始化控制器】//
bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) 
{ 
  setlocale(LC_ALL,"");//避免日志中中文乱码 

  // 1-1. 从配置文件读取关节名
  std::vector<std::string> joint_names;
  if (!nh.getParam("joint_names", joint_names) || joint_names.size() != 8) {
    ROS_ERROR("请在配置文件中设置8个关节名!");
    return false;
  }

  // 1-2. 获取8个关节的控制句柄
  for (const auto& name : joint_names) {
    joint_handles_.push_back(hw->getHandle(name));
  }

  //1-3. 初始化8个PID控制器（从配置文件加载参数）
  pid_controllers_.resize(8);
  for (int i = 0; i < 8; ++i) {
    std::string pid_param = "pid/" + joint_names[i];
    if (!pid_controllers_[i].init(ros::NodeHandle(nh, pid_param))) {
      ROS_ERROR("加载关节[%s]的PID参数失败!", joint_names[i].c_str());
      return false;
    }
  }

  // 1-4. 初始化动态调参服务器（支持rqt_reconfigure），要在堆上new否则，init后服务器就被销毁了
  dyn_server.reset(new dynamic_reconfigure::Server<sentry_chassis_controller::ChassisPIDConfig>(nh));
  // // 绑定回调函数：把 dynParamCallback 绑定到服务器
  // // 用 std::bind 把成员函数和 this 指针绑定（关键！让服务器能找到这个函数）
  // dyn_cbType = std::bind(&SentryChassisController::dynParamCallback, 
  //                         this,  // 必须传 this 指针，告诉函数是哪个类的实例
  //                         std::placeholders::_1,  // 占位符：对应 config 参数
  //                         std::placeholders::_2); // 占位符：对应 level 参数
  // // 给服务器设置回调函数                        
  // dyn_server->setCallback(dyn_cbType);


  // 1-5. 初始化期望速度（默认0）
  target_motor.resize(4, 0.0);
  target_servo.resize(4, 0.0);
  ROS_INFO("控制器初始化成功，加载了8个关节的PID！");

  //1-6.逆运动学
  // 读取底盘参数（从配置文件.yaml读取，无则用默认值）
  nh.param<double>("wheel_radius", wheel_radius_, 0.009);        // 轮子半径默认0.1m
  nh.param<double>("wheelbase", wheelbase_, 0.362);             // 轴距默认0.5m
  nh.param<double>("track_width", track_width_,  0.362);         // 轮距默认0.4m
  nh.param<double>("max_steer_angle", max_steer_angle_, 1.57); // 舵机最大角度默认±90度（1.57rad）
  // 订阅/cmd_vel话题（接收速度指令，队列大小10）
  cmd_vel_sub_ = nh.subscribe("/mycmd_vel", 10, &SentryChassisController::cmdVelCallback, this);
  // 初始化期望速度/角度（保持不变，默认0）
  ROS_INFO("控制器初始化成功！已加载逆运动学，支持/cmd_vel指令");
  ROS_INFO("底盘参数：轮径=%.4fm，轴距=%.4fm，轮距=%.4fm", wheel_radius_, wheelbase_, track_width_);
  
  //1-7.初始化里程计
  this->initOdom(nh);

  //1-8. 新增：初始化速度模式和TF监听
  this->initVelMode(nh);
  // TF监听器初始化（必须在tf_buffer_之后创建，且用智能指针避免生命周期问题）
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
  ROS_INFO("速度模式初始化完成！当前模式：%s", is_global_vel_mode_ ? "全局坐标系" : "底盘坐标系");


  return true;  // 必须返回true，否则控制器加载失败
}

//【二.更新】//
void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
  static int count = 0;
  const int print_interval = 200;  // 100Hz控制频率 → 每100帧（1秒）打印一次
  if (++count % print_interval == 0) {
     ROS_INFO("controller is running now!!!!!!!"); 
     ROS_INFO("target参数:电机1=%.2f,电机2=%.2f,电机3=%.2f,电机4=%.2f",target_motor[0],target_motor[1],target_motor[2],target_motor[3]);
     ROS_INFO("target参数:舵机1=%.2f,舵机2=%.2f,舵机3=%.2f,舵机4=%.2f",target_servo[0],target_servo[1],target_servo[2],target_servo[3]);
  }
  // //2-1.读取动态参数
  //this->dynParamRead();
  // //2-2.新增：调用里程计更新（周期100Hz，与控制频率一致）
  this->updateOdom(time, period); 
  //2-3.遍历8个关节，逐个执行PID控制
  for (int i = 0; i < 4; ++i) 
  {
    // 1. 读取关节当前电机角速度，舵机以一个固定点为起始转过角度
    double current_vel = joint_handles_[i].getVelocity();
    double current_position =joint_handles_[i+4].getPosition();
    // 2. 计算速度误差（期望速度 - 当前速度）
    double error_v =target_motor[i] - current_vel; //8- current_vel;//
    ROS_INFO("目标转速：%.2f;当前转速：%.2f",target_motor[i] ,current_vel);
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

//【三.动态参数】//参数读取函数（读取参数服务器的参数，更新PID和目标值）
void SentryChassisController::dynParamRead() 
{
  // 定义临时变量存储读取的参数（避免直接修改本地变量导致的中间状态错误）
  double Kp_motor, Ki_motor, Kd_motor, i_clamp_motor;
  double Kp_servo, Ki_servo, Kd_servo, i_clamp_servo;
  double target_motor_all, target_servo_all;

  // 从参数服务器读取参数（参数名要和 .cfg 完全一致！）
  // 注意：如果控制器有命名空间，需在参数名前加命名空间（比如 "sentry_controller/Kp_motor"）
  ros::NodeHandle nh("/controller/sentry_chassis_controller"); // 若有命名空间，改为 ros::NodeHandle nh("sentry_controller");

  // 读取电机组参数（.cfg中定义的参数名）
  nh.param<double>("Kp_motor", Kp_motor, 2.0);          // 第二个参数是默认值（参数不存在时用）
  nh.param<double>("Ki_motor", Ki_motor, 0.1);
  nh.param<double>("Kd_motor", Kd_motor, 0.0);
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
    //target_motor[i] = target_motor_all;//rqt_plot调pid用的
    pid_controllers_[i+4].initPid(Kp_servo, Ki_servo, Kd_servo, i_clamp_servo, -i_clamp_servo);
    //target_servo[i] = target_servo_all;
  }
}

//【四.逆运动】//
//工具函数：限制val在[min_val, max_val]之间（避免舵机超程）
template<typename T>
T SentryChassisController::clamp(T val, T min_val, T max_val) {
  return std::max(min_val, std::min(val, max_val));
}

//cmd_vel回调函数：逆运动学解算（将底盘速度指令 → 每个轮子的转向角+转速）
void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  
  // 新增：根据模式判断是否需要转换速度
  geometry_msgs::Twist target_vel; // 最终传入逆运动学的速度（底盘坐标系）
  if (is_global_vel_mode_) {
    // 全局坐标系模式：将/cmd_vel（世界坐标）转换为底盘坐标
    if (!this->transformWorldVelToBaseVel(msg, target_vel)) {
      // 转换失败时，使用原速度（降级为底盘坐标系）
      target_vel = *msg;
      ROS_WARN("全局速度转换失败，降级使用底盘坐标系速度！");
    } else {
      //转换成功
      ROS_INFO_THROTTLE(1, "全局→底盘速度转换：world(vx=%.2f, vy=%.2f) → base(vx=%.2f, vy=%.2f)，yaw=%.2frad",
                       msg->linear.x, msg->linear.y, target_vel.linear.x, target_vel.linear.y, yaw_from_tf_);
    }
  } else {
    // 底盘坐标系模式：直接使用原速度
    target_vel = *msg;
  }
  
  const double v_x = target_vel.linear.x;
  const double v_y = target_vel.linear.y;
  const double omega_z = target_vel.angular.z;

  // 轮子坐标（轮子相对底盘中心坐标）
  const double LF_x = wheelbase_ / 2.0;
  const double LF_y = track_width_ / 2.0;
  const double RF_x = wheelbase_ / 2.0;
  const double RF_y = -track_width_ / 2.0;
  const double LR_x = -wheelbase_ / 2.0;
  const double LR_y = track_width_ / 2.0;
  const double RR_x = -wheelbase_ / 2.0;
  const double RR_y = -track_width_ / 2.0;


  double LF_steer = 0.0, RF_steer = 0.0, RR_steer = 0.0, LR_steer = 0.0;
  double LF_vel = 0.0, RF_vel = 0.0, RR_vel = 0.0, LR_vel = 0.0;

  if (fabs(omega_z) < 1e-6) //即w==0
  {
    // (1)纯平移
    if (fabs(v_x) < 1e-6 && fabs(v_y) < 1e-6) //即v_x==0&&v_y==0
    {
      LF_steer = RF_steer = RR_steer = LR_steer = 0.0;
    } else {
      const double move_angle = atan2(v_y, v_x);
      LF_steer = move_angle;//舵机左正右负
      RF_steer = move_angle;
      LR_steer = move_angle;
      RR_steer = move_angle;
    }
    const double linear_vel = sqrt(v_x*v_x + v_y*v_y);
    LF_vel = linear_vel / wheel_radius_;
    RF_vel = linear_vel / wheel_radius_;
    LR_vel = linear_vel / wheel_radius_;
    RR_vel = linear_vel / wheel_radius_;
  } 
  else //(2)w!=0时
  {
    //逆运动求解核心！！！
    const double ICC_x = -v_y / omega_z;//轨迹圆心点相对于底盘中心点坐标
    const double ICC_y =  v_x / omega_z;//本质是r=v/w
    const double LF_vy = -(ICC_x-LF_x)*omega_z;//已知：轮->底盘中心；圆心->底盘中心的相对坐标，底盘中心的vx,vy,w
    const double LF_vx =  (ICC_y-LF_y)*omega_z;//就可以求出轮的vx,vy  v=r*w(核心就是求轮与圆心的距离r)
    const double RF_vy = -(ICC_x-RF_x)*omega_z;
    const double RF_vx =  (ICC_y-RF_y)*omega_z;
    const double LR_vy = -(ICC_x-LR_x)*omega_z;
    const double LR_vx =  (ICC_y-LR_y)*omega_z;
    const double RR_vy = -(ICC_x-RR_x)*omega_z;
    const double RR_vx =  (ICC_y-RR_y)*omega_z;
    
    // (2-1). 计算每个轮子的舵机转向角
    LF_steer = atan2(LF_vy,LF_vx);
    RF_steer = atan2(RF_vy,RF_vx);
    LR_steer = atan2(LR_vy,LR_vx);
    RR_steer = atan2(RR_vy,RR_vx);
    // (2-2). 计算每个轮子转速
    LF_vel = sqrt(LF_vx*LF_vx + LF_vy*LF_vy) / wheel_radius_;
    RF_vel = sqrt(RF_vx*RF_vx + RF_vy*RF_vy) / wheel_radius_;
    LR_vel = sqrt(LR_vx*LR_vx + LR_vy*LR_vy) / wheel_radius_;
    RR_vel = sqrt(RR_vx*RR_vx + RR_vy*RR_vy) / wheel_radius_;
  }

  // 角度归一化+转速修正（复用逻辑，减少冗余），防止舵机转过头，让电机反转即可
  auto normalize_angle_vel = [](double& steer, double& vel) {
    if (steer > M_PI/2) {
      steer -= M_PI;
      vel = -vel;
    } else if (steer < -M_PI/2) {
      steer += M_PI;
      vel = -vel;
    }
  };
  // 调用函数（替代重复的if-else）
  normalize_angle_vel(LF_steer, LF_vel);
  normalize_angle_vel(RF_steer, RF_vel);
  normalize_angle_vel(LR_steer, LR_vel);
  normalize_angle_vel(RR_steer, RR_vel);

  // 设置最大转向角
  max_steer_angle_ = M_PI / 2.0; // 1.57rad
  LF_steer = clamp(LF_steer, -max_steer_angle_, max_steer_angle_);
  RF_steer = clamp(RF_steer, -max_steer_angle_, max_steer_angle_);
  RR_steer = clamp(RR_steer, -max_steer_angle_, max_steer_angle_);
  LR_steer = clamp(LR_steer, -max_steer_angle_, max_steer_angle_);

  // 更新目标值（适配舵机“左正右负”）
  target_servo[0] = LF_steer;  // 左前：+0.785rad（左转45°）
  target_servo[1] = RF_steer;  // 右前：-0.785rad（右转45°）
  target_servo[2] = LR_steer;  // 左后：-0.785rad（右转45°）
  target_servo[3] = RR_steer;  // 右后：+0.785rad（左转45°）

  target_motor[0] = LF_vel;   // 左前：正（逆时针）
  target_motor[1] = RF_vel;   // 右前：负（顺时针）
  target_motor[2] = LR_vel;   // 左后：负（顺时针）
  target_motor[3] = RR_vel;   // 右后：正（逆时针）
  //ROS_INFO("发布转速：%.2f;得到转速：%.2f",LF_vel,joint_handles_[0].getVelocity());
}

//【五.里程计与tf坐标变换】//
//新增：里程计初始化函数
void SentryChassisController::initOdom(ros::NodeHandle& nh) {
  // 1. 创建odom话题发布者（队列大小10， latch=true确保新节点能获取历史数据）
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10, true);

  // 2. 初始化里程计消息的固定字段（无需周期修改）
  odom_msg_.header.frame_id = "odom";        // 父坐标系：odom
  odom_msg_.child_frame_id = "base_link";     // 子坐标系：base_link（底盘中心）
  odom_msg_.pose.pose.position.x = 0.0;       // 初始x坐标
  odom_msg_.pose.pose.position.y = 0.0;       // 初始y坐标
  odom_msg_.pose.pose.position.z = 0.0;       // 2D里程计，z恒为0
  // 初始姿态：无旋转，四元数为(0,0,0,1)//即 “欧拉角”（比如航向角、俯仰角、横滚角）
  odom_quat_.setRPY(0.0, 0.0, 0.0);
  odom_msg_.pose.pose.orientation.x = odom_quat_.x();
  odom_msg_.pose.pose.orientation.y = odom_quat_.y();
  odom_msg_.pose.pose.orientation.z = odom_quat_.z();
  odom_msg_.pose.pose.orientation.w = odom_quat_.w();

  // 3. 初始化时间戳（记录首次初始化时间）
  last_odom_time_ = ros::Time::now();

  ROS_INFO("里程计初始化成功！将发布 /odom 话题和 odom→base_link 坐标变换");
}
// 新增：里程计周期更新函数（正运动学解算和里程计更新）
void SentryChassisController::updateOdom(const ros::Time& time, const ros::Duration& period) {
  // 1. 读取4个轮子的实际速度（从关节句柄获取，索引0-3是电机）
  double LF_vel = joint_handles_[0].getVelocity();  // 左前电机实际角速度（rad/s）
  double RF_vel = joint_handles_[1].getVelocity();  // 右前电机实际角速度
  double LR_vel = joint_handles_[2].getVelocity();  // 左后电机实际角速度
  double RR_vel = joint_handles_[3].getVelocity();  // 右后电机实际角速度
  ROS_INFO("得到转速：%.2f",joint_handles_[0].getVelocity());

  // 2. 轮子角速度 → 线速度（v = ω × r）
  double LF_lin_vel = LF_vel * wheel_radius_;
  double RF_lin_vel = RF_vel * wheel_radius_;
  double LR_lin_vel = LR_vel * wheel_radius_;
  double RR_lin_vel = RR_vel * wheel_radius_;

  // 3. 正运动学解算：由轮子线速度 → 底盘速度（vx, vy, ωz）  
  const double LF_x = wheelbase_ / 2.0;
  const double LF_y = track_width_ / 2.0;
  const double RF_x = wheelbase_ / 2.0;
  const double RF_y = -track_width_ / 2.0;
  const double LR_x = -wheelbase_ / 2.0;
  const double LR_y = track_width_ / 2.0;
  const double RR_x = -wheelbase_ / 2.0;
  const double RR_y = -track_width_ / 2.0;

  double LF_vel_x=LF_lin_vel*cos(joint_handles_[4].getPosition());//轮子线速度分量
  double LF_vel_y=LF_lin_vel*sin(joint_handles_[4].getPosition());
  double RF_vel_x=RF_lin_vel*cos(joint_handles_[5].getPosition());
  double RF_vel_y=RF_lin_vel*sin(joint_handles_[5].getPosition());
  double LR_vel_x=LR_lin_vel*cos(joint_handles_[6].getPosition());
  double LR_vel_y=LR_lin_vel*sin(joint_handles_[6].getPosition());
  double RR_vel_x=RR_lin_vel*cos(joint_handles_[7].getPosition());
  double RR_vel_y=RR_lin_vel*sin(joint_handles_[7].getPosition());
  //正运动学解算核心!!!
  double vx=(LF_vel_x+RF_vel_x+LR_vel_x+RR_vel_x)/4;//底盘中心速度
  double vy=(LF_vel_y+RF_vel_y+LR_vel_y+RR_vel_y)/4;
  double omega_z_LF =-(vy-LF_vel_y)/LF_x;
  double omega_z_RF =-(vy-RF_vel_y)/RF_x;
  double omega_z_LR =-(vy-LR_vel_y)/LR_x;
  double omega_z_RR =-(vy-RR_vel_y)/RR_x;//const double ICC_x = -v_y / omega_z;//轨迹圆心点相对于底盘中心点坐标
  //由这两个公式推导而出：                  //const double LF_vy = -(ICC_x-LF_x)*omega_z;
  double omega_z=(omega_z_LF+omega_z_RF+omega_z_LR+omega_z_RR)/4;
  ROS_INFO("正运动求解:v_x=%.2f;v_y=%.2f;w=%.2f",vx,vy,omega_z);

  // 4. 计算位移和姿态变化（积分：速度 × 时间）
  double dt = period.toSec();  // 时间间隔（控制器更新周期，默认100Hz→0.01s）
  //理解：odom_th_是底盘x轴相对odom的x轴的角度！！！
  double delta_x = (vx * cos(odom_th_) + vy * cos(odom_th_+ M_PI / 2)) * dt;  // x方向位移（考虑航向角）
  double delta_y = (vx * sin(odom_th_) + vy * sin(odom_th_+ M_PI / 2)) * dt;  // y方向位移
  double delta_th = omega_z * dt;  // 航向角变化（z轴旋转）

  // 5. 更新里程计累计坐标和姿态
  odom_x_ += delta_x;
  odom_y_ += delta_y;
  odom_th_ += delta_th;  // 范围可限制在[-π, π]，避免数值溢出

  // 6. 姿态角（odom_th_）→ 四元数（ROS里程计要求用四元数表示姿态）
  odom_quat_.setRPY(0.0, 0.0, odom_th_); //odom_th_没有范围限制 ，2D里程计，滚转/俯仰为0，仅航向角有效
  this->yaw_from_tf_=odom_th_;
  // 7. 填充里程计消息
  odom_msg_.header.stamp = time;
  // 位置信息
  odom_msg_.pose.pose.position.x = odom_x_;
  odom_msg_.pose.pose.position.y = odom_y_;
  // 姿态信息（四元数）：累计的位置 / 姿态（pose 字段）
  odom_msg_.pose.pose.orientation.x = odom_quat_.x();
  odom_msg_.pose.pose.orientation.y = odom_quat_.y();
  odom_msg_.pose.pose.orientation.z = odom_quat_.z();
  odom_msg_.pose.pose.orientation.w = odom_quat_.w();
  // 速度信息（正运动学解算的底盘速度）：实时的速度（twist 字段）
  odom_msg_.twist.twist.linear.x = vx;
  odom_msg_.twist.twist.linear.y = vy;
  odom_msg_.twist.twist.angular.z = omega_z;

  // 8. 发布里程计消息
  odom_pub_.publish(odom_msg_);//发布的是累计的位置 / 姿态,实时的速度；需要tf转化为实时的位置 / 姿态！！！

  // 9. 广播odom→base_link的坐标变换（rviz可视化需要）
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = time;
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";
  // 变换位置
  tf_msg.transform.translation.x = odom_x_;
  tf_msg.transform.translation.y = odom_y_;
  tf_msg.transform.translation.z = 0.0;
  // 变换姿态（四元数）
  tf_msg.transform.rotation.x = odom_quat_.x();
  tf_msg.transform.rotation.y = odom_quat_.y();
  tf_msg.transform.rotation.z = odom_quat_.z();
  tf_msg.transform.rotation.w = odom_quat_.w();
  // 发布坐标变换
  tf_broadcaster_.sendTransform(tf_msg);//发布的是转化后实时的两坐标系之间的位置，姿态信息
  //TF 系统的设计是 “广播 - 监听” 模式：没有专门话题，只要监听者“监听 TF 变换”，就可以收到！！！
}


//【六.odom坐标系下的逆运动】//
// 新增：从配置文件读取速度模式（在.yaml中配置mode: global/base）
void SentryChassisController::initVelMode(ros::NodeHandle& nh) {
  std::string vel_mode;
  // 从配置文件读取模式，默认"base"（底盘坐标系）
  if (nh.getParam("vel_mode", vel_mode) && vel_mode == "global") {
    is_global_vel_mode_ = true;
  } else {
    is_global_vel_mode_ = false;
    // 若未配置或配置错误，打印警告并使用默认模式
    ROS_WARN("未配置vel_mode或配置错误，默认使用底盘坐标系模式！可在配置文件中设置vel_mode: global");
  }
}
// 新增：核心！！！：将世界坐标系速度转换为底盘坐标系速度
bool SentryChassisController::transformWorldVelToBaseVel(const geometry_msgs::Twist::ConstPtr& world_vel, 
                                                        geometry_msgs::Twist& base_vel) {
  try {
    // 1. 从TF缓存中获取odom→base_link的最新变换（ros::Time(0)表示最新时刻）
    // geometry_msgs::TransformStamped odom_to_base_tf = 
    //   tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));
    
    // // 2. 从TF变换中提取航向角yaw（四元数→欧拉角）
    // tf2::Quaternion quat;//四元数
    // tf2::fromMsg(odom_to_base_tf.transform.rotation, quat);
    // double roll, pitch, yaw;//欧拉角
    // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 仅关注yaw（绕z轴旋转角）
    double yaw=yaw_from_tf_; // 保存航向角，用于日志打印
    
    // 3. 应用速度转换公式（世界坐标系→底盘坐标系）
    base_vel.linear.x =  world_vel->linear.x * cos(yaw) + world_vel->linear.y * sin(yaw);
    base_vel.linear.y = -world_vel->linear.x * sin(yaw) + world_vel->linear.y * cos(yaw);
    base_vel.angular.z = world_vel->angular.z; // 旋转角速度不变
    
    return true; // 转换成功
  } catch (tf2::TransformException& ex) {
    // TF获取失败（如变换未发布、超时），打印错误并返回失败
    ROS_ERROR("TF变换获取失败：%s（可能odom→base_link未发布）", ex.what());
    return false;
  }
}



}  // namespace sentry_chassis_controller

// 注册控制器为ROS插件（关键！让controller_manager能找到这个控制器）
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
