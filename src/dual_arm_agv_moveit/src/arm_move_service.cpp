#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "dual_arm_agv_moveit/srv/move_arm.hpp"
#include <thread>
#include <future>

class ArmMoveService : public rclcpp::Node
{
public:
  ArmMoveService() : Node("arm_move_service")
  {
    service_ = this->create_service<dual_arm_agv_moveit::srv::MoveArm>(
      "move_arm",
      std::bind(&ArmMoveService::handle_move_arm, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "🤖 机械臂移动服务已启动，等待调用...");
    RCLCPP_INFO(this->get_logger(), "📋 支持的控制模式:");
    RCLCPP_INFO(this->get_logger(), "   - arm_r: 仅控制右臂");
    RCLCPP_INFO(this->get_logger(), "   - arm_l: 仅控制左臂");
    RCLCPP_INFO(this->get_logger(), "   - both: 同时控制双臂");
  }

private:
  struct MoveResult
  {
    bool success;
    std::string message;
  };

  // 单臂移动函数
  MoveResult move_single_arm(const std::string& arm_group, 
                             const geometry_msgs::msg::Pose& target_pose)
  {
    MoveResult result;
    
    try
    {
      RCLCPP_INFO(this->get_logger(), "🔧 初始化 %s MoveIt 接口...", arm_group.c_str());
      
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), arm_group);

      // 打印当前位置
      auto current_pose = move_group.getCurrentPose();
      RCLCPP_INFO(this->get_logger(), 
                  "📍 %s 当前位置: x=%.3f y=%.3f z=%.3f",
                  arm_group.c_str(),
                  current_pose.pose.position.x,
                  current_pose.pose.position.y,
                  current_pose.pose.position.z);

      // 打印目标位置
      RCLCPP_INFO(this->get_logger(), 
                  "🎯 %s 目标位置: x=%.3f y=%.3f z=%.3f",
                  arm_group.c_str(),
                  target_pose.position.x,
                  target_pose.position.y,
                  target_pose.position.z);

      // 设置目标位姿
      move_group.setPoseTarget(target_pose);

      // 规划路径
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group.plan(plan) == 
                          moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (plan_success)
      {
        RCLCPP_INFO(this->get_logger(), "✅ %s 规划成功，正在执行...", arm_group.c_str());
        
        // 执行规划
        auto exec_result = move_group.execute(plan);
        
        if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "✅ %s 成功移动到目标位置", arm_group.c_str());
          result.success = true;
          result.message = arm_group + " 成功移动到目标位置";
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "⚠️ %s 执行失败", arm_group.c_str());
          result.success = false;
          result.message = arm_group + " 执行失败";
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "❌ %s 规划失败", arm_group.c_str());
        result.success = false;
        result.message = arm_group + " 规划失败，目标可能不可达";
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ %s 异常: %s", arm_group.c_str(), e.what());
      result.success = false;
      result.message = arm_group + " 发生异常: " + std::string(e.what());
    }
    
    return result;
  }

  void handle_move_arm(
    const std::shared_ptr<dual_arm_agv_moveit::srv::MoveArm::Request> request,
    std::shared_ptr<dual_arm_agv_moveit::srv::MoveArm::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "📨 收到移动请求，控制模式: %s", 
                request->control_mode.c_str());

    response->right_arm_success = false;
    response->left_arm_success = false;
    response->success = false;

    if (request->control_mode == "arm_r")
    {
      // 仅控制右臂
      RCLCPP_INFO(this->get_logger(), "🦾 开始控制右臂...");
      auto result = move_single_arm("arm_r", request->target_pose_right);
      response->right_arm_success = result.success;
      response->success = result.success;
      response->message = result.message;
    }
    else if (request->control_mode == "arm_l")
    {
      // 仅控制左臂
      RCLCPP_INFO(this->get_logger(), "🦾 开始控制左臂...");
      auto result = move_single_arm("arm_l", request->target_pose_left);  // 修改：这里改成 arm_l
      response->left_arm_success = result.success;
      response->success = result.success;
      response->message = result.message;
    }
    else if (request->control_mode == "both")
    {
      // 同时控制双臂（使用多线程并行执行）
      RCLCPP_INFO(this->get_logger(), "🦾🦾 开始同时控制双臂...");
      
      // 创建异步任务
      auto future_right = std::async(std::launch::async, 
                                     &ArmMoveService::move_single_arm, 
                                     this, 
                                     "arm_r", 
                                     request->target_pose_right);
      
      auto future_left = std::async(std::launch::async, 
                                    &ArmMoveService::move_single_arm, 
                                    this, 
                                    "arm_l", 
                                    request->target_pose_left);
      
      // 等待两个任务完成
      auto result_right = future_right.get();
      auto result_left = future_left.get();
      
      response->right_arm_success = result_right.success;
      response->left_arm_success = result_left.success;
      response->success = result_right.success && result_left.success;
      
      // 组合消息
      response->message = "右臂: " + result_right.message + 
                         "; 左臂: " + result_left.message;
      
      if (response->success)
      {
        RCLCPP_INFO(this->get_logger(), "✅ 双臂均成功移动到目标位置");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "⚠️ 至少有一个机械臂未成功移动");
      }
    }
    else
    {
      // 无效的控制模式
      RCLCPP_ERROR(this->get_logger(), 
                   "❌ 无效的控制模式: %s (应为 'arm_r', 'arm_l' 或 'both')", 
                   request->control_mode.c_str());
      response->success = false;
      response->message = "无效的控制模式: " + request->control_mode;
    }
  }

  rclcpp::Service<dual_arm_agv_moveit::srv::MoveArm>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmMoveService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}