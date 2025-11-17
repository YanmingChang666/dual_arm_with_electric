#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_right_arm_node");

  // 初始化 MoveIt 接口
  moveit::planning_interface::MoveGroupInterface move_group(node, "r_arm");  // 改成你右臂的group名称

  // 打印当前末端位置
  auto current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(node->get_logger(), "当前末端位置: x=%.3f y=%.3f z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // 设置目标位置
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation = current_pose.pose.orientation;  // 保持当前姿态
  target_pose.position.x = 0.0;
  target_pose.position.y = -0.5;
  target_pose.position.z = 0.97;

  move_group.setPoseTarget(target_pose);

  // 规划路径
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "✅ 规划成功，正在执行...");
    auto exec_result = move_group.execute(plan);
    if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node->get_logger(), "✅ 右臂成功移动到目标位置");
    else
      RCLCPP_WARN(node->get_logger(), "⚠️ 执行失败");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "❌ 规划失败，请检查目标是否可达");
  }

  rclcpp::shutdown();
  return 0;
}

