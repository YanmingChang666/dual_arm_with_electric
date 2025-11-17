#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/msg/pose.hpp>
#include "dual_arm_with_electric_moveit/srv/move_arm.hpp"
#include <thread>
#include <future>

class ArmMoveServiceIKFast : public rclcpp::Node
{
public:
  ArmMoveServiceIKFast() : Node("arm_move_service_ikfast")
  {
    // 参数配置
    this->declare_parameter("use_custom_ik", true);
    this->declare_parameter("ik_timeout", 0.05);  // IKFast很快，50ms足够
    this->declare_parameter("num_waypoints", 10);  // 路径点数量
    this->declare_parameter("lift_height", 0.1);   // 抬起高度（米）
    this->declare_parameter("ik_attempts", 10);    // IK尝试次数
    
    service_ = this->create_service<dual_arm_with_electric_moveit::srv::MoveArm>(
      "move_arm",
      std::bind(&ArmMoveServiceIKFast::handle_move_arm, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "🤖 机械臂移动服务已启动（IKFast模式）");
    RCLCPP_INFO(this->get_logger(), "⚡ 使用自定义IK求解器进行精确控制");
    RCLCPP_INFO(this->get_logger(), "📐 路径规划：垂直提升 → 水平平移");
  }

private:
  struct MoveResult
  {
    bool success;
    std::string message;
  };

  // 🆕 生成保持水平的路径点
  std::vector<geometry_msgs::msg::Pose> generate_horizontal_waypoints(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& arm_group)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    int num_waypoints = this->get_parameter("num_waypoints").as_int();
    
    RCLCPP_INFO(this->get_logger(), 
                "📍 生成 %d 个路径点，保持姿态不变...", num_waypoints);
    
    // 阶段1：垂直抬起（保持xy不变，只改变z）
    int lift_points = num_waypoints / 3;
    for (int i = 1; i <= lift_points; ++i)
    {
      geometry_msgs::msg::Pose wp = start_pose;
      double t = static_cast<double>(i) / lift_points;
      
      // 线性插值高度
      wp.position.z = start_pose.position.z + 
                      t * (target_pose.position.z - start_pose.position.z);
      
      // 姿态保持不变（使用目标姿态）
      wp.orientation = target_pose.orientation;
      
      waypoints.push_back(wp);
    }
    
    RCLCPP_INFO(this->get_logger(), "   ⬆️  阶段1: 垂直抬起 %d 个点", lift_points);
    
    // 阶段2：水平平移（保持z不变，改变xy）
    int translate_points = num_waypoints - lift_points;
    for (int i = 1; i <= translate_points; ++i)
    {
      geometry_msgs::msg::Pose wp;
      double t = static_cast<double>(i) / translate_points;
      
      // 线性插值xy位置
      wp.position.x = start_pose.position.x + 
                      t * (target_pose.position.x - start_pose.position.x);
      wp.position.y = start_pose.position.y + 
                      t * (target_pose.position.y - start_pose.position.y);
      wp.position.z = target_pose.position.z;  // 保持目标高度
      
      // 姿态保持目标姿态（水平）
      wp.orientation = target_pose.orientation;
      
      waypoints.push_back(wp);
    }
    
    RCLCPP_INFO(this->get_logger(), "   ➡️  阶段2: 水平平移 %d 个点", translate_points);
    
    return waypoints;
  }

  // 🆕 使用IK求解器验证并执行路径
  MoveResult move_with_custom_ik(
    const std::string& arm_group,
    const geometry_msgs::msg::Pose& target_pose)
  {
    MoveResult result;
    
    try
    {
      RCLCPP_INFO(this->get_logger(), "🔧 初始化 %s MoveGroupInterface...", 
                  arm_group.c_str());
      
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), arm_group);
      
      // 获取当前位姿
      auto current_pose = move_group.getCurrentPose().pose;
      
      RCLCPP_INFO(this->get_logger(), 
                  "📍 当前位置: [%.3f, %.3f, %.3f]",
                  current_pose.position.x,
                  current_pose.position.y,
                  current_pose.position.z);
      
      RCLCPP_INFO(this->get_logger(), 
                  "🎯 目标位置: [%.3f, %.3f, %.3f]",
                  target_pose.position.x,
                  target_pose.position.y,
                  target_pose.position.z);
      
      // 生成路径点
      auto waypoints = generate_horizontal_waypoints(
        current_pose, target_pose, arm_group);
      
      // 🔑 关键：使用自定义IK验证每个路径点
      bool use_custom_ik = this->get_parameter("use_custom_ik").as_bool();
      
      if (use_custom_ik)
      {
        result = execute_with_ik_validation(move_group, waypoints, arm_group);
      }
      else
      {
        // 回退到笛卡尔路径规划
        result = execute_cartesian_path(move_group, waypoints, arm_group);
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 异常: %s", e.what());
      result.success = false;
      result.message = "异常: " + std::string(e.what());
    }
    
    return result;
  }

  // 🆕 使用IK验证并执行路径
  MoveResult execute_with_ik_validation(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& arm_group)
  {
    MoveResult result;
    
    RCLCPP_INFO(this->get_logger(), "🧮 使用IK求解器验证路径点...");
    
    // 加载机器人模型
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    const moveit::core::RobotModelPtr& kinematic_model = 
      robot_model_loader.getModel();
    
    moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    
    const moveit::core::JointModelGroup* joint_model_group = 
      kinematic_model->getJointModelGroup(arm_group);
    
    if (!joint_model_group)
    {
      result.success = false;
      result.message = "找不到关节组: " + arm_group;
      return result;
    }
    
    double ik_timeout = this->get_parameter("ik_timeout").as_double();
    int ik_attempts = this->get_parameter("ik_attempts").as_int();
    
    // 验证每个路径点的IK可达性
    int valid_count = 0;
    std::vector<std::vector<double>> joint_solutions;
    
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      bool found_ik = kinematic_state->setFromIK(
        joint_model_group,
        waypoints[i],
        ik_timeout,
        moveit::core::GroupStateValidityCallbackFn(),
        kinematics::KinematicsQueryOptions()
      );
      
      if (found_ik)
      {
        valid_count++;
        
        // 保存关节角度
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        joint_solutions.push_back(joint_values);
        
        if (i % 5 == 0)  // 每5个点打印一次
        {
          RCLCPP_INFO(this->get_logger(), 
                      "   ✅ 路径点 %zu/%zu IK求解成功", 
                      i+1, waypoints.size());
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), 
                    "   ⚠️  路径点 %zu/%zu IK求解失败", 
                    i+1, waypoints.size());
      }
    }
    
    double success_rate = static_cast<double>(valid_count) / waypoints.size();
    RCLCPP_INFO(this->get_logger(), 
                "📊 IK验证完成: %d/%zu (%.1f%%) 路径点可达",
                valid_count, waypoints.size(), success_rate * 100.0);
    
    // 如果大部分路径点可达，执行笛卡尔路径
    if (success_rate >= 0.9)  // 90%以上可达
    {
      RCLCPP_INFO(this->get_logger(), "✅ 路径可行，开始执行笛卡尔路径...");
      result = execute_cartesian_path(move_group, waypoints, arm_group);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), 
                   "❌ 路径不可行（成功率%.1f%% < 90%%）", 
                   success_rate * 100.0);
      result.success = false;
      result.message = "IK验证失败，路径不可达";
    }
    
    return result;
  }

  // 执行笛卡尔路径
  MoveResult execute_cartesian_path(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& arm_group)
  {
    MoveResult result;
    
    RCLCPP_INFO(this->get_logger(), "📐 计算笛卡尔路径...");
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;  // 1cm步长
    double jump_threshold = 0.0;
    
    double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);
    
    RCLCPP_INFO(this->get_logger(), 
                "📊 笛卡尔路径完成度: %.2f%%", fraction * 100.0);
    
    if (fraction >= 0.95)
    {
      RCLCPP_INFO(this->get_logger(), "✅ 路径规划成功，开始执行...");
      
      // 执行轨迹
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      
      auto exec_result = move_group.execute(plan);
      
      if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "✅ %s 执行成功！", arm_group.c_str());
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
      RCLCPP_ERROR(this->get_logger(), 
                   "❌ 路径规划失败（完成度%.2f%%）", fraction * 100.0);
      result.success = false;
      result.message = arm_group + " 路径规划失败";
    }
    
    return result;
  }

  void handle_move_arm(
    const std::shared_ptr<dual_arm_with_electric_moveit::srv::MoveArm::Request> request,
    std::shared_ptr<dual_arm_with_electric_moveit::srv::MoveArm::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "📨 收到移动请求: %s", 
                request->control_mode.c_str());

    response->right_arm_success = false;
    response->left_arm_success = false;
    response->success = false;

    if (request->control_mode == "r_arm")
    {
      RCLCPP_INFO(this->get_logger(), "🦾 控制右臂（IKFast模式）");
      auto result = move_with_custom_ik("r_arm", request->target_pose_right);
      response->right_arm_success = result.success;
      response->success = result.success;
      response->message = result.message;
    }
    else if (request->control_mode == "l_arm")
    {
      RCLCPP_INFO(this->get_logger(), "🦾 控制左臂（IKFast模式）");
      auto result = move_with_custom_ik("l_arm", request->target_pose_left);
      response->left_arm_success = result.success;
      response->success = result.success;
      response->message = result.message;
    }
    else if (request->control_mode == "both_arms")
    {
      RCLCPP_INFO(this->get_logger(), "🦾🦾 同时控制双臂（IKFast模式）");
      
      auto future_right = std::async(std::launch::async, 
                                     &ArmMoveServiceIKFast::move_with_custom_ik, 
                                     this, "r_arm", 
                                     request->target_pose_right);
      
      auto future_left = std::async(std::launch::async, 
                                    &ArmMoveServiceIKFast::move_with_custom_ik, 
                                    this, "l_arm", 
                                    request->target_pose_left);
      
      auto result_right = future_right.get();
      auto result_left = future_left.get();
      
      response->right_arm_success = result_right.success;
      response->left_arm_success = result_left.success;
      response->success = result_right.success && result_left.success;
      response->message = "右臂: " + result_right.message + 
                         "; 左臂: " + result_left.message;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 无效的控制模式: %s", 
                   request->control_mode.c_str());
      response->success = false;
      response->message = "无效的控制模式";
    }
  }

  rclcpp::Service<dual_arm_with_electric_moveit::srv::MoveArm>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmMoveServiceIKFast>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}