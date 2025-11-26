#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

class PlanningDiagnostic : public rclcpp::Node
{
public:
  PlanningDiagnostic() : Node("planning_diagnostic")
  {
    this->declare_parameter("arm_group", "arm_l");
    this->declare_parameter("target_x", 0.26087);
    this->declare_parameter("target_y", -0.87515);
    this->declare_parameter("target_z", 1.31267);
    
    // 使用定时器延迟执行
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PlanningDiagnostic::diagnose, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  bool executed_ = false;
  void diagnose()
  {
    // 确保只执行一次
    if (executed_)
    {
      return;
    }
    executed_ = true;
    
    // 取消定时器
    timer_->cancel();
    
    std::string arm_group = this->get_parameter("arm_group").as_string();
    
    RCLCPP_INFO(this->get_logger(), "🔧 开始诊断 %s 规划问题...", arm_group.c_str());
    
    try
    {
      // 1. 检查 MoveIt 接口
      RCLCPP_INFO(this->get_logger(), "\n========== 步骤 1: 检查 MoveIt 接口 ==========");
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), arm_group);
      RCLCPP_INFO(this->get_logger(), "✅ MoveIt 接口初始化成功");
      
      // 2. 检查规划组信息
      RCLCPP_INFO(this->get_logger(), "\n========== 步骤 2: 规划组信息 ==========");
      RCLCPP_INFO(this->get_logger(), "规划组名称: %s", move_group.getName().c_str());
      RCLCPP_INFO(this->get_logger(), "末端执行器: %s", 
                  move_group.getEndEffectorLink().c_str());
      RCLCPP_INFO(this->get_logger(), "参考坐标系: %s", 
                  move_group.getPoseReferenceFrame().c_str());
      RCLCPP_INFO(this->get_logger(), "规划时间: %.1f 秒", 
                  move_group.getPlanningTime());
      
      // 3. 检查关节限制
      RCLCPP_INFO(this->get_logger(), "\n========== 步骤 3: 关节信息 ==========");
      auto joint_names = move_group.getJointNames();
      RCLCPP_INFO(this->get_logger(), "关节数量: %zu", joint_names.size());
      for (const auto& name : joint_names)
      {
        RCLCPP_INFO(this->get_logger(), "  - %s", name.c_str());
      }
      
      // 4. 检查当前状态
      RCLCPP_INFO(this->get_logger(), "\n========== 步骤 4: 当前状态 ==========");
      
      // 等待 joint_states
      rclcpp::sleep_for(std::chrono::seconds(1));
      
      auto current_pose = move_group.getCurrentPose();
      RCLCPP_INFO(this->get_logger(), "当前末端位置:");
      RCLCPP_INFO(this->get_logger(), "  X: %.4f", current_pose.pose.position.x);
      RCLCPP_INFO(this->get_logger(), "  Y: %.4f", current_pose.pose.position.y);
      RCLCPP_INFO(this->get_logger(), "  Z: %.4f", current_pose.pose.position.z);
      
      auto current_joints = move_group.getCurrentJointValues();
      RCLCPP_INFO(this->get_logger(), "当前关节角度:");
      for (size_t i = 0; i < current_joints.size(); ++i)
      {
        RCLCPP_INFO(this->get_logger(), "  关节 %zu: %.4f rad (%.2f°)", 
                    i, current_joints[i], current_joints[i] * 180.0 / M_PI);
      }
      
      // 5. 测试目标位置
      RCLCPP_INFO(this->get_logger(), "\n========== 步骤 5: 测试目标位置 ==========");
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = this->get_parameter("target_x").as_double();
      target_pose.position.y = this->get_parameter("target_y").as_double();
      target_pose.position.z = this->get_parameter("target_z").as_double();
      target_pose.orientation.x = 0.5;
      target_pose.orientation.y = -0.5;
      target_pose.orientation.z = -0.5;
      target_pose.orientation.w = 0.5;
      
      RCLCPP_INFO(this->get_logger(), "目标位置:");
      RCLCPP_INFO(this->get_logger(), "  X: %.4f", target_pose.position.x);
      RCLCPP_INFO(this->get_logger(), "  Y: %.4f", target_pose.position.y);
      RCLCPP_INFO(this->get_logger(), "  Z: %.4f", target_pose.position.z);
      
      // 计算距离
      double distance = sqrt(
        pow(target_pose.position.x - current_pose.pose.position.x, 2) +
        pow(target_pose.position.y - current_pose.pose.position.y, 2) +
        pow(target_pose.position.z - current_pose.pose.position.z, 2)
      );
      RCLCPP_INFO(this->get_logger(), "移动距离: %.4f 米", distance);
      
      // 6. 尝试多次规划
      RCLCPP_INFO(this->get_logger(), "\n========== 步骤 6: 尝试规划 ==========");
      
      move_group.setPoseTarget(target_pose);
      move_group.setPlanningTime(10.0);
      move_group.setNumPlanningAttempts(10);
      
      int success_count = 0;
      int total_attempts = 5;
      
      for (int i = 0; i < total_attempts; ++i)
      {
        RCLCPP_INFO(this->get_logger(), "\n尝试 %d/%d:", i+1, total_attempts);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group.plan(plan);
        
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "  ✅ 规划成功!");
          RCLCPP_INFO(this->get_logger(), "  轨迹点数: %zu", 
                      plan.trajectory_.joint_trajectory.points.size());
          RCLCPP_INFO(this->get_logger(), "  预计时间: %.2f 秒",
                      plan.trajectory_.joint_trajectory.points.back().time_from_start.sec);
          success_count++;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "  ❌ 规划失败");
        }
      }
      
      // 7. 输出诊断结果
      RCLCPP_INFO(this->get_logger(), "\n========== 诊断结果 ==========");
      RCLCPP_INFO(this->get_logger(), "规划成功率: %d/%d (%.1f%%)", 
                  success_count, total_attempts, 
                  100.0 * success_count / total_attempts);
      
      if (success_count == 0)
      {
        RCLCPP_ERROR(this->get_logger(), "\n❌ 所有规划尝试都失败了!");
        RCLCPP_INFO(this->get_logger(), "\n可能的原因:");
        RCLCPP_INFO(this->get_logger(), "1. 目标位置超出工作空间");
        RCLCPP_INFO(this->get_logger(), "2. 目标姿态不可达");
        RCLCPP_INFO(this->get_logger(), "3. 存在碰撞");
        RCLCPP_INFO(this->get_logger(), "4. 运动学求解器配置问题");
        RCLCPP_INFO(this->get_logger(), "5. joint_states 数据问题");
        
        RCLCPP_INFO(this->get_logger(), "\n💡 建议:");
        RCLCPP_INFO(this->get_logger(), "1. 检查 joint_states 话题: ros2 topic echo /joint_states");
        RCLCPP_INFO(this->get_logger(), "2. 检查 kinematics.yaml 配置");
        RCLCPP_INFO(this->get_logger(), "3. 在 RViz 中可视化目标位置");
        RCLCPP_INFO(this->get_logger(), "4. 尝试更接近当前位置的目标");
        RCLCPP_INFO(this->get_logger(), "5. 使用 find_min_z_position 搜索可达位置");
      }
      else if (success_count < total_attempts)
      {
        RCLCPP_WARN(this->get_logger(), "\n⚠️ 规划成功率较低");
        RCLCPP_INFO(this->get_logger(), "建议增加规划时间或尝试次数");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "\n✅ 规划器工作正常!");
      }
      
      // 执行完成后请求关闭节点
      rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 诊断过程中发生异常: %s", e.what());
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningDiagnostic>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}