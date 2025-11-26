#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <iomanip>

class FindMinZUniversal : public rclcpp::Node
{
public:
  FindMinZUniversal() : Node("find_min_z_universal")
  {
    // 声明参数
    this->declare_parameter("arm_group", "arm_l");
    
    // XY 位置参数
    this->declare_parameter("test_x", 0.26087);
    this->declare_parameter("test_y", -0.87515);
    
    // Z 搜索范围参数
    this->declare_parameter("start_z", 1.5);
    this->declare_parameter("min_z", 0.3);
    this->declare_parameter("z_step", 0.02);
    
    // 姿态参数 - 支持两种输入方式
    // 方式1: 直接指定四元数
    this->declare_parameter("use_quaternion", true);
    this->declare_parameter("quat_x", 0.5);
    this->declare_parameter("quat_y", -0.5);
    this->declare_parameter("quat_z", -0.5);
    this->declare_parameter("quat_w", 0.5);
    
    // 方式2: 使用欧拉角(roll, pitch, yaw) 单位:度
    this->declare_parameter("roll_deg", 0.0);
    this->declare_parameter("pitch_deg", 0.0);
    this->declare_parameter("yaw_deg", 0.0);
    
    // 规划参数
    this->declare_parameter("planning_time", 5.0);
    this->declare_parameter("planning_attempts", 10);
    this->declare_parameter("consecutive_fail_threshold", 3);
    
    // 输出参数
    this->declare_parameter("verbose", true);  // 是否详细输出每次测试结果
    
    // 使用定时器延迟执行
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FindMinZUniversal::find_minimum_z, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  bool executed_ = false;

  // 从欧拉角创建四元数
  geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();
    
    return quat_msg;
  }
  
  // 从四元数提取欧拉角(用于显示)
  void quaternion_to_euler(const geometry_msgs::msg::Quaternion& q, 
                          double& roll, double& pitch, double& yaw)
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    m.getRPY(roll, pitch, yaw);
  }

  void find_minimum_z()
  {
    // 确保只执行一次
    if (executed_)
    {
      return;
    }
    executed_ = true;
    timer_->cancel();
    
    // 获取参数
    std::string arm_group = this->get_parameter("arm_group").as_string();
    double test_x = this->get_parameter("test_x").as_double();
    double test_y = this->get_parameter("test_y").as_double();
    double start_z = this->get_parameter("start_z").as_double();
    double min_z = this->get_parameter("min_z").as_double();
    double z_step = this->get_parameter("z_step").as_double();
    
    bool use_quaternion = this->get_parameter("use_quaternion").as_bool();
    bool verbose = this->get_parameter("verbose").as_bool();
    
    double planning_time = this->get_parameter("planning_time").as_double();
    int planning_attempts = this->get_parameter("planning_attempts").as_int();
    int fail_threshold = this->get_parameter("consecutive_fail_threshold").as_int();
    
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║   通用 Z 轴最低点搜索工具                     ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "");
    
    try
    {
      // 初始化 MoveIt
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), arm_group);
      
      // 设置规划参数
      move_group.setPlanningTime(planning_time);
      move_group.setNumPlanningAttempts(planning_attempts);
      move_group.setMaxVelocityScalingFactor(0.5);
      move_group.setMaxAccelerationScalingFactor(0.5);
      
      RCLCPP_INFO(this->get_logger(), "✅ MoveIt 接口初始化成功");
      RCLCPP_INFO(this->get_logger(), "   规划组: %s", arm_group.c_str());
      RCLCPP_INFO(this->get_logger(), "   末端执行器: %s", 
                  move_group.getEndEffectorLink().c_str());
      RCLCPP_INFO(this->get_logger(), "   参考坐标系: %s\n", 
                  move_group.getPoseReferenceFrame().c_str());
      
      // 获取当前状态
      auto current_pose = move_group.getCurrentPose();
      RCLCPP_INFO(this->get_logger(), "📍 当前末端位置:");
      RCLCPP_INFO(this->get_logger(), "   X: %.4f m", current_pose.pose.position.x);
      RCLCPP_INFO(this->get_logger(), "   Y: %.4f m", current_pose.pose.position.y);
      RCLCPP_INFO(this->get_logger(), "   Z: %.4f m\n", current_pose.pose.position.z);
      
      // 准备目标位姿
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = test_x;
      target_pose.position.y = test_y;
      
      // 设置姿态
      if (use_quaternion)
      {
        target_pose.orientation.x = this->get_parameter("quat_x").as_double();
        target_pose.orientation.y = this->get_parameter("quat_y").as_double();
        target_pose.orientation.z = this->get_parameter("quat_z").as_double();
        target_pose.orientation.w = this->get_parameter("quat_w").as_double();
        
        RCLCPP_INFO(this->get_logger(), "🎯 测试配置:");
        RCLCPP_INFO(this->get_logger(), "   位置: X=%.4f, Y=%.4f", test_x, test_y);
        RCLCPP_INFO(this->get_logger(), "   姿态(四元数): [%.3f, %.3f, %.3f, %.3f]",
                    target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
        
        // 同时显示对应的欧拉角
        double roll, pitch, yaw;
        quaternion_to_euler(target_pose.orientation, roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "   姿态(欧拉角): Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
      }
      else
      {
        double roll_deg = this->get_parameter("roll_deg").as_double();
        double pitch_deg = this->get_parameter("pitch_deg").as_double();
        double yaw_deg = this->get_parameter("yaw_deg").as_double();
        
        // 转换为弧度
        double roll = roll_deg * M_PI / 180.0;
        double pitch = pitch_deg * M_PI / 180.0;
        double yaw = yaw_deg * M_PI / 180.0;
        
        target_pose.orientation = euler_to_quaternion(roll, pitch, yaw);
        
        RCLCPP_INFO(this->get_logger(), "🎯 测试配置:");
        RCLCPP_INFO(this->get_logger(), "   位置: X=%.4f, Y=%.4f", test_x, test_y);
        RCLCPP_INFO(this->get_logger(), "   姿态(欧拉角): Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°",
                    roll_deg, pitch_deg, yaw_deg);
        RCLCPP_INFO(this->get_logger(), "   姿态(四元数): [%.3f, %.3f, %.3f, %.3f]",
                    target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
      }
      
      RCLCPP_INFO(this->get_logger(), "   Z搜索范围: %.3f → %.3f (步长: %.3f)\n", 
                  start_z, min_z, z_step);
      
      // 存储结果
      struct TestResult
      {
        double z;
        bool success;
        int trajectory_points;
        double planning_time;
      };
      
      std::vector<TestResult> results;
      std::vector<double> valid_z_positions;
      double lowest_valid_z = start_z;
      bool found_any_valid = false;
      int consecutive_fails = 0;
      
      RCLCPP_INFO(this->get_logger(), "🔍 开始 Z 轴搜索...\n");
      RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      
      // 从上往下搜索
      int test_count = 0;
      for (double z = start_z; z >= min_z; z -= z_step)
      {
        test_count++;
        target_pose.position.z = z;
        
        if (verbose)
        {
          RCLCPP_INFO(this->get_logger(), "测试 #%d: Z = %.4f m", test_count, z);
        }
        
        move_group.setPoseTarget(target_pose);
        
        // 记录规划时间
        auto start_time = std::chrono::high_resolution_clock::now();
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_success = (move_group.plan(plan) == 
                            moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(end_time - start_time).count();
        
        TestResult result;
        result.z = z;
        result.success = plan_success;
        result.planning_time = elapsed;
        
        if (plan_success)
        {
          result.trajectory_points = plan.trajectory_.joint_trajectory.points.size();
          
          if (verbose)
          {
            RCLCPP_INFO(this->get_logger(), 
                       "   ✅ 可达! (轨迹点: %d, 规划耗时: %.3fs)", 
                       result.trajectory_points, elapsed);
          }
          
          valid_z_positions.push_back(z);
          lowest_valid_z = z;
          found_any_valid = true;
          consecutive_fails = 0;
        }
        else
        {
          result.trajectory_points = 0;
          
          if (verbose)
          {
            RCLCPP_WARN(this->get_logger(), "   ❌ 不可达 (规划耗时: %.3fs)", elapsed);
          }
          
          // 连续失败检测
          if (found_any_valid)
          {
            consecutive_fails++;
            if (consecutive_fails >= fail_threshold)
            {
              RCLCPP_INFO(this->get_logger(), 
                         "\n⚠️  连续 %d 次规划失败,停止搜索(已找到最低点)", 
                         fail_threshold);
              break;
            }
          }
        }
        
        results.push_back(result);
      }
      
      RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
      
      // 输出详细统计
      RCLCPP_INFO(this->get_logger(), "");
      RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════╗");
      RCLCPP_INFO(this->get_logger(), "║                  搜索结果统计                  ║");
      RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
      RCLCPP_INFO(this->get_logger(), "");
      
      if (found_any_valid)
      {
        RCLCPP_INFO(this->get_logger(), "✅ 测试总数: %d", test_count);
        RCLCPP_INFO(this->get_logger(), "✅ 可达位置: %zu 个", valid_z_positions.size());
        RCLCPP_INFO(this->get_logger(), "✅ 成功率: %.1f%%", 
                    100.0 * valid_z_positions.size() / test_count);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🎯 关键结果:");
        RCLCPP_INFO(this->get_logger(), "   最高可达 Z: %.4f m", valid_z_positions.front());
        RCLCPP_INFO(this->get_logger(), "   最低可达 Z: %.4f m", lowest_valid_z);
        RCLCPP_INFO(this->get_logger(), "   可达范围: %.4f m", 
                    valid_z_positions.front() - lowest_valid_z);
        RCLCPP_INFO(this->get_logger(), "");
        
        // 显示部分可达位置
        RCLCPP_INFO(this->get_logger(), "📋 可达 Z 位置列表(前10个):");
        for (size_t i = 0; i < std::min(size_t(10), valid_z_positions.size()); ++i)
        {
          RCLCPP_INFO(this->get_logger(), "   %2zu. Z = %.4f m", 
                     i+1, valid_z_positions[i]);
        }
        
        if (valid_z_positions.size() > 10)
        {
          RCLCPP_INFO(this->get_logger(), "   ... (还有 %zu 个)", 
                     valid_z_positions.size() - 10);
          RCLCPP_INFO(this->get_logger(), "   %2zu. Z = %.4f m (最低)", 
                     valid_z_positions.size(), lowest_valid_z);
        }
        
        RCLCPP_INFO(this->get_logger(), "");
        
        // 生成 ROS2 服务调用命令
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "📝 使用最低位置的 ROS2 服务调用命令:");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "");
        
        std::stringstream ss;
        ss << "ros2 service call /move_arm dual_arm_agv_moveit/srv/MoveArm \"{\n"
           << "  control_mode: '" << arm_group << "',\n"
           << "  target_pose_" << (arm_group == "arm_l" ? "left" : "right") << ": {\n"
           << "    position: {x: " << std::fixed << std::setprecision(5) << test_x 
           << ", y: " << test_y << ", z: " << lowest_valid_z << "},\n"
           << "    orientation: {x: " << std::setprecision(3) << target_pose.orientation.x
           << ", y: " << target_pose.orientation.y
           << ", z: " << target_pose.orientation.z
           << ", w: " << target_pose.orientation.w << "}\n"
           << "  }\n"
           << "}\"";
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "");
        
        // 生成 YAML 格式配置
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "📄 YAML 格式配置(可保存到文件):");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "target_pose:");
        RCLCPP_INFO(this->get_logger(), "  position:");
        RCLCPP_INFO(this->get_logger(), "    x: %.5f", test_x);
        RCLCPP_INFO(this->get_logger(), "    y: %.5f", test_y);
        RCLCPP_INFO(this->get_logger(), "    z: %.5f  # 最低可达", lowest_valid_z);
        RCLCPP_INFO(this->get_logger(), "  orientation:");
        RCLCPP_INFO(this->get_logger(), "    x: %.3f", target_pose.orientation.x);
        RCLCPP_INFO(this->get_logger(), "    y: %.3f", target_pose.orientation.y);
        RCLCPP_INFO(this->get_logger(), "    z: %.3f", target_pose.orientation.z);
        RCLCPP_INFO(this->get_logger(), "    w: %.3f", target_pose.orientation.w);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "❌ 在搜索范围内未找到任何可达位置!");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "💡 建议:");
        RCLCPP_INFO(this->get_logger(), "   1. 增大起始高度 (start_z)");
        RCLCPP_INFO(this->get_logger(), "   2. 调整 X,Y 位置");
        RCLCPP_INFO(this->get_logger(), "   3. 更换目标姿态(orientation)");
        RCLCPP_INFO(this->get_logger(), "   4. 检查机器人工作空间限制");
        RCLCPP_INFO(this->get_logger(), "   5. 增加规划时间或尝试次数");
        RCLCPP_INFO(this->get_logger(), "   6. 在 RViz 中可视化目标位置");
      }
      
      RCLCPP_INFO(this->get_logger(), "");
      RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
      RCLCPP_INFO(this->get_logger(), "");
      
      // 完成后关闭
      rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 发生异常: %s", e.what());
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FindMinZUniversal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}