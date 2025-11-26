#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

class FindMinZPosition : public rclcpp::Node
{
public:
  FindMinZPosition() : Node("find_min_z_position")
  {
    // 声明参数
    this->declare_parameter("arm_group", "arm_l");
    this->declare_parameter("base_x", 0.26087);
    this->declare_parameter("base_y", -0.87515);
    this->declare_parameter("start_z", 1.5);  // 起始高度
    this->declare_parameter("min_z", 0.5);    // 最小搜索高度
    this->declare_parameter("z_step", 0.02);  // Z轴搜索步长
    this->declare_parameter("orientation_x", 0.0);
    this->declare_parameter("orientation_y", 0.0);
    this->declare_parameter("orientation_z", 0.0);
    this->declare_parameter("orientation_w", 1.0);
    
    // 使用定时器延迟执行,确保 shared_ptr 完全创建
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FindMinZPosition::find_minimum_z, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  bool executed_ = false;
  void find_minimum_z()
  {
    // 确保只执行一次
    if (executed_)
    {
      return;
    }
    executed_ = true;
    
    // 取消定时器
    timer_->cancel();
    
    // 获取参数
    std::string arm_group = this->get_parameter("arm_group").as_string();
    double base_x = this->get_parameter("base_x").as_double();
    double base_y = this->get_parameter("base_y").as_double();
    double start_z = this->get_parameter("start_z").as_double();
    double min_z = this->get_parameter("min_z").as_double();
    double z_step = this->get_parameter("z_step").as_double();
    
    RCLCPP_INFO(this->get_logger(), "🔍 开始搜索 %s 的最低可达Z位置...", arm_group.c_str());
    RCLCPP_INFO(this->get_logger(), "📍 X,Y 固定位置: (%.3f, %.3f)", base_x, base_y);
    RCLCPP_INFO(this->get_logger(), "📏 搜索范围: Z = %.3f 到 %.3f, 步长 = %.3f", 
                start_z, min_z, z_step);
    
    try
    {
      // 初始化 MoveIt
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), arm_group);
      
      // 设置规划参数
      move_group.setPlanningTime(5.0);
      move_group.setNumPlanningAttempts(10);
      move_group.setMaxVelocityScalingFactor(0.5);
      move_group.setMaxAccelerationScalingFactor(0.5);
      
      RCLCPP_INFO(this->get_logger(), "✅ MoveIt 接口初始化成功");
      
      // 获取当前状态
      auto current_pose = move_group.getCurrentPose();
      RCLCPP_INFO(this->get_logger(), "📍 当前位置: x=%.3f y=%.3f z=%.3f",
                  current_pose.pose.position.x,
                  current_pose.pose.position.y,
                  current_pose.pose.position.z);
      
      // 准备目标位姿
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = base_x;
      target_pose.position.y = base_y;
      target_pose.orientation.x = this->get_parameter("orientation_x").as_double();
      target_pose.orientation.y = this->get_parameter("orientation_y").as_double();
      target_pose.orientation.z = this->get_parameter("orientation_z").as_double();
      target_pose.orientation.w = this->get_parameter("orientation_w").as_double();
      
      // 存储可达位置
      std::vector<double> valid_z_positions;
      double lowest_valid_z = start_z;
      bool found_any_valid = false;
      int consecutive_fails = 0;
      
      // 从上往下搜索
      for (double z = start_z; z >= min_z; z -= z_step)
      {
        target_pose.position.z = z;
        
        RCLCPP_INFO(this->get_logger(), "🎯 测试位置: (%.3f, %.3f, %.3f)", 
                    base_x, base_y, z);
        
        move_group.setPoseTarget(target_pose);
        
        // 尝试规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_success = (move_group.plan(plan) == 
                            moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (plan_success)
        {
          RCLCPP_INFO(this->get_logger(), "   ✅ Z = %.3f 可达!", z);
          valid_z_positions.push_back(z);
          lowest_valid_z = z;
          found_any_valid = true;
          consecutive_fails = 0;  // 重置失败计数
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "   ❌ Z = %.3f 不可达", z);
          
          // 如果已经找到可达位置,连续3次失败后停止
          if (found_any_valid)
          {
            consecutive_fails++;
            if (consecutive_fails >= 3)
            {
              RCLCPP_INFO(this->get_logger(), 
                         "⚠️ 连续3次规划失败,停止搜索(已找到最低点)");
              break;
            }
          }
        }
      }
      
      // 输出结果
      RCLCPP_INFO(this->get_logger(), "\n========== 搜索结果 ==========");
      if (found_any_valid)
      {
        RCLCPP_INFO(this->get_logger(), "✅ 找到 %zu 个可达位置", valid_z_positions.size());
        RCLCPP_INFO(this->get_logger(), "🎯 最低可达 Z 位置: %.3f", lowest_valid_z);
        RCLCPP_INFO(this->get_logger(), "📋 完整可达位置列表:");
        for (size_t i = 0; i < valid_z_positions.size(); ++i)
        {
          RCLCPP_INFO(this->get_logger(), "   %zu. Z = %.3f", i+1, valid_z_positions[i]);
        }
        
        // 输出可以直接使用的服务调用命令
        RCLCPP_INFO(this->get_logger(), "\n📝 使用最低位置的服务调用命令:");
        RCLCPP_INFO(this->get_logger(), "ros2 service call /move_arm dual_arm_agv_moveit/srv/MoveArm \"{"
                                       "\n  control_mode: '%s',"
                                       "\n  target_pose_%s: {"
                                       "\n    position: {x: %.5f, y: %.5f, z: %.5f},"
                                       "\n    orientation: {x: %.1f, y: %.1f, z: %.1f, w: %.1f}"
                                       "\n  }"
                                       "\n}\"",
                    arm_group.c_str(),
                    arm_group == "arm_l" ? "left" : "right",
                    base_x, base_y, lowest_valid_z,
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "❌ 在搜索范围内未找到任何可达位置!");
        RCLCPP_INFO(this->get_logger(), "💡 建议:");
        RCLCPP_INFO(this->get_logger(), "   1. 增大起始高度 (start_z)");
        RCLCPP_INFO(this->get_logger(), "   2. 调整 X,Y 位置");
        RCLCPP_INFO(this->get_logger(), "   3. 更换目标姿态");
        RCLCPP_INFO(this->get_logger(), "   4. 检查机器人工作空间限制");
      }
      RCLCPP_INFO(this->get_logger(), "==============================\n");
      
      // 执行完成后请求关闭节点
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
  auto node = std::make_shared<FindMinZPosition>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}