#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <algorithm>
#include <limits>

class GlobalMinZFinder : public rclcpp::Node
{
public:
  GlobalMinZFinder() : Node("global_min_z_finder")
  {
    // 声明参数
    this->declare_parameter("arm_group", "arm_l");
    
    // XY 搜索范围
    this->declare_parameter("x_min", 0.15);
    this->declare_parameter("x_max", 0.45);
    this->declare_parameter("x_step", 0.05);
    this->declare_parameter("y_min", -1.0);
    this->declare_parameter("y_max", -0.6);
    this->declare_parameter("y_step", 0.05);
    
    // Z 搜索范围
    this->declare_parameter("z_start", 1.5);
    this->declare_parameter("z_min", 0.2);
    this->declare_parameter("z_step", 0.02);
    
    // 规划参数
    this->declare_parameter("planning_time", 3.0);
    this->declare_parameter("planning_attempts", 5);
    
    // 预定义姿态(四元数)
    this->declare_parameter("test_orientations", true);  // 是否测试多个姿态
    
    // 使用定时器延迟执行
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GlobalMinZFinder::find_global_min_z, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  bool executed_ = false;
  
  // 预定义的常用姿态
  struct Orientation
  {
    std::string name;
    double x, y, z, w;
  };
  
  std::vector<Orientation> get_test_orientations()
  {
    std::vector<Orientation> orientations;
    
    // 1. 垂直向下 (最常用的抓取姿态)
    orientations.push_back({"vertical_down", 0.5, -0.5, -0.5, 0.5});
    
    // 2. 45度倾斜
    orientations.push_back({"tilt_45deg", 0.383, -0.383, -0.383, 0.707});
    
    // 3. 30度倾斜
    orientations.push_back({"tilt_30deg", 0.259, -0.259, -0.259, 0.866});
    
    // 4. 水平向前
    orientations.push_back({"horizontal_forward", 0.0, 0.707, -0.707, 0.0});
    
    // 5. 侧向
    orientations.push_back({"side_grasp", 0.707, 0.0, 0.0, 0.707});
    
    return orientations;
  }
  
  struct TestResult
  {
    double x, y, z;
    std::string orientation_name;
    bool success;
    double planning_time;
    
    TestResult() : x(0), y(0), z(0), success(false), planning_time(0) {}
  };
  
  // 测试单个位置和姿态的最低Z
  TestResult test_position_orientation(
    moveit::planning_interface::MoveGroupInterface& move_group,
    double x, double y, const Orientation& orient,
    double z_start, double z_min, double z_step)
  {
    TestResult result;
    result.x = x;
    result.y = y;
    result.orientation_name = orient.name;
    result.success = false;
    
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.orientation.x = orient.x;
    target_pose.orientation.y = orient.y;
    target_pose.orientation.z = orient.z;
    target_pose.orientation.w = orient.w;
    
    double lowest_z = z_start;
    bool found_any = false;
    int consecutive_fails = 0;
    
    // 从上往下搜索Z
    for (double z = z_start; z >= z_min; z -= z_step)
    {
      target_pose.position.z = z;
      move_group.setPoseTarget(target_pose);
      
      auto start_time = std::chrono::high_resolution_clock::now();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group.plan(plan) == 
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);
      auto end_time = std::chrono::high_resolution_clock::now();
      
      if (success)
      {
        lowest_z = z;
        found_any = true;
        consecutive_fails = 0;
        result.planning_time += std::chrono::duration<double>(end_time - start_time).count();
      }
      else
      {
        if (found_any)
        {
          consecutive_fails++;
          if (consecutive_fails >= 3)
          {
            break;  // 连续失败,停止搜索
          }
        }
      }
    }
    
    if (found_any)
    {
      result.z = lowest_z;
      result.success = true;
    }
    
    return result;
  }

  void find_global_min_z()
  {
    if (executed_) return;
    executed_ = true;
    timer_->cancel();
    
    // 获取参数
    std::string arm_group = this->get_parameter("arm_group").as_string();
    double x_min = this->get_parameter("x_min").as_double();
    double x_max = this->get_parameter("x_max").as_double();
    double x_step = this->get_parameter("x_step").as_double();
    double y_min = this->get_parameter("y_min").as_double();
    double y_max = this->get_parameter("y_max").as_double();
    double y_step = this->get_parameter("y_step").as_double();
    double z_start = this->get_parameter("z_start").as_double();
    double z_min = this->get_parameter("z_min").as_double();
    double z_step = this->get_parameter("z_step").as_double();
    double planning_time = this->get_parameter("planning_time").as_double();
    int planning_attempts = this->get_parameter("planning_attempts").as_int();
    bool test_orientations = this->get_parameter("test_orientations").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "\n");
    RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║        全局最低 Z 坐标自动搜索工具                  ║");
    RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "");
    
    try
    {
      // 初始化 MoveIt
      moveit::planning_interface::MoveGroupInterface move_group(
        shared_from_this(), arm_group);
      
      move_group.setPlanningTime(planning_time);
      move_group.setNumPlanningAttempts(planning_attempts);
      move_group.setMaxVelocityScalingFactor(0.5);
      move_group.setMaxAccelerationScalingFactor(0.5);
      
      RCLCPP_INFO(this->get_logger(), "✅ MoveIt 接口初始化成功");
      RCLCPP_INFO(this->get_logger(), "   规划组: %s", arm_group.c_str());
      RCLCPP_INFO(this->get_logger(), "   末端执行器: %s\n", 
                  move_group.getEndEffectorLink().c_str());
      
      // 生成测试位置
      std::vector<std::pair<double, double>> test_positions;
      for (double x = x_min; x <= x_max; x += x_step)
      {
        for (double y = y_min; y <= y_max; y += y_step)
        {
          test_positions.push_back({x, y});
        }
      }
      
      // 获取测试姿态
      std::vector<Orientation> orientations;
      if (test_orientations)
      {
        orientations = get_test_orientations();
      }
      else
      {
        // 只使用默认垂直向下姿态
        orientations.push_back({"vertical_down", 0.5, -0.5, -0.5, 0.5});
      }
      
      int total_tests = test_positions.size() * orientations.size();
      
      RCLCPP_INFO(this->get_logger(), "🔍 搜索配置:");
      RCLCPP_INFO(this->get_logger(), "   X 范围: [%.2f, %.2f] 步长: %.3f (共 %d 点)",
                  x_min, x_max, x_step, 
                  (int)((x_max - x_min) / x_step) + 1);
      RCLCPP_INFO(this->get_logger(), "   Y 范围: [%.2f, %.2f] 步长: %.3f (共 %d 点)",
                  y_min, y_max, y_step,
                  (int)((y_max - y_min) / y_step) + 1);
      RCLCPP_INFO(this->get_logger(), "   Z 范围: [%.2f, %.2f] 步长: %.3f",
                  z_start, z_min, z_step);
      RCLCPP_INFO(this->get_logger(), "   测试姿态: %zu 种", orientations.size());
      RCLCPP_INFO(this->get_logger(), "   总测试数: %d\n", total_tests);
      
      if (orientations.size() > 1)
      {
        RCLCPP_INFO(this->get_logger(), "📐 测试姿态列表:");
        for (const auto& o : orientations)
        {
          RCLCPP_INFO(this->get_logger(), "   - %s: [%.3f, %.3f, %.3f, %.3f]",
                      o.name.c_str(), o.x, o.y, o.z, o.w);
        }
        RCLCPP_INFO(this->get_logger(), "");
      }
      
      RCLCPP_INFO(this->get_logger(), "⏳ 开始搜索... (预计需要 %d-%d 分钟)\n",
                  total_tests * 2 / 60, total_tests * 5 / 60);
      RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      
      // 存储所有结果
      std::vector<TestResult> all_results;
      TestResult global_best;
      global_best.z = std::numeric_limits<double>::max();
      global_best.success = false;
      
      int test_count = 0;
      int success_count = 0;
      auto search_start_time = std::chrono::high_resolution_clock::now();
      
      // 遍历所有位置和姿态组合
      for (const auto& orient : orientations)
      {
        RCLCPP_INFO(this->get_logger(), "\n🎯 测试姿态: %s", orient.name.c_str());
        
        for (const auto& pos : test_positions)
        {
          test_count++;
          
          // 测试这个位置和姿态
          auto result = test_position_orientation(
            move_group, pos.first, pos.second, orient,
            z_start, z_min, z_step);
          
          if (result.success)
          {
            all_results.push_back(result);
            success_count++;
            
            RCLCPP_INFO(this->get_logger(), 
                       "   [%3d/%3d] ✅ (%.2f, %.2f) → Z_min = %.4f m",
                       test_count, total_tests, pos.first, pos.second, result.z);
            
            // 更新全局最优
            if (result.z < global_best.z)
            {
              global_best = result;
              RCLCPP_INFO(this->get_logger(), 
                         "   🎉 新的全局最低点! Z = %.4f m", result.z);
            }
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), 
                       "   [%3d/%3d] ❌ (%.2f, %.2f) → 不可达",
                       test_count, total_tests, pos.first, pos.second);
          }
          
          // 每10个测试输出一次进度
          if (test_count % 10 == 0)
          {
            double progress = 100.0 * test_count / total_tests;
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(
              current_time - search_start_time).count();
            double estimated_total = elapsed * total_tests / test_count;
            double remaining = estimated_total - elapsed;
            
            RCLCPP_INFO(this->get_logger(), 
                       "\n   📊 进度: %.1f%% (%d/%d) | 已用时: %.1f分钟 | 预计剩余: %.1f分钟",
                       progress, test_count, total_tests, 
                       elapsed / 60.0, remaining / 60.0);
            
            if (global_best.success)
            {
              RCLCPP_INFO(this->get_logger(), 
                         "   🏆 当前最低: Z = %.4f m @ (%.2f, %.2f) [%s]\n",
                         global_best.z, global_best.x, global_best.y,
                         global_best.orientation_name.c_str());
            }
          }
        }
      }
      
      auto search_end_time = std::chrono::high_resolution_clock::now();
      double total_time = std::chrono::duration<double>(
        search_end_time - search_start_time).count();
      
      RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
      
      // 输出最终结果
      RCLCPP_INFO(this->get_logger(), "");
      RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════╗");
      RCLCPP_INFO(this->get_logger(), "║                    搜索完成!                         ║");
      RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════╝");
      RCLCPP_INFO(this->get_logger(), "");
      
      RCLCPP_INFO(this->get_logger(), "📊 统计信息:");
      RCLCPP_INFO(this->get_logger(), "   总测试数: %d", test_count);
      RCLCPP_INFO(this->get_logger(), "   成功数: %d", success_count);
      RCLCPP_INFO(this->get_logger(), "   成功率: %.1f%%", 
                  100.0 * success_count / test_count);
      RCLCPP_INFO(this->get_logger(), "   总耗时: %.1f 分钟\n", total_time / 60.0);
      
      if (global_best.success)
      {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "🏆 全局最低 Z 坐标:");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "   位置:");
        RCLCPP_INFO(this->get_logger(), "      X = %.4f m", global_best.x);
        RCLCPP_INFO(this->get_logger(), "      Y = %.4f m", global_best.y);
        RCLCPP_INFO(this->get_logger(), "      Z = %.4f m  ⬅️ 最低点!", global_best.z);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "   姿态: %s", 
                    global_best.orientation_name.c_str());
        
        // 找到对应的姿态详细信息
        for (const auto& o : orientations)
        {
          if (o.name == global_best.orientation_name)
          {
            RCLCPP_INFO(this->get_logger(), "      四元数: [%.3f, %.3f, %.3f, %.3f]",
                       o.x, o.y, o.z, o.w);
            break;
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "📝 ROS2 服务调用命令:");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "");
        
        // 找到姿态四元数
        double qx, qy, qz, qw;
        for (const auto& o : orientations)
        {
          if (o.name == global_best.orientation_name)
          {
            qx = o.x; qy = o.y; qz = o.z; qw = o.w;
            break;
          }
        }
        
        RCLCPP_INFO(this->get_logger(),
          "ros2 service call /move_arm dual_arm_agv_moveit/srv/MoveArm \"{\n"
          "  control_mode: '%s',\n"
          "  target_pose_%s: {\n"
          "    position: {x: %.5f, y: %.5f, z: %.5f},\n"
          "    orientation: {x: %.3f, y: %.3f, z: %.3f, w: %.3f}\n"
          "  }\n"
          "}\"",
          arm_group.c_str(),
          arm_group == "arm_l" ? "left" : "right",
          global_best.x, global_best.y, global_best.z,
          qx, qy, qz, qw);
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "📄 YAML 格式配置:");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "global_min_z_pose:");
        RCLCPP_INFO(this->get_logger(), "  position:");
        RCLCPP_INFO(this->get_logger(), "    x: %.5f", global_best.x);
        RCLCPP_INFO(this->get_logger(), "    y: %.5f", global_best.y);
        RCLCPP_INFO(this->get_logger(), "    z: %.5f  # 全局最低点", global_best.z);
        RCLCPP_INFO(this->get_logger(), "  orientation:");
        RCLCPP_INFO(this->get_logger(), "    x: %.3f", qx);
        RCLCPP_INFO(this->get_logger(), "    y: %.3f", qy);
        RCLCPP_INFO(this->get_logger(), "    z: %.3f", qz);
        RCLCPP_INFO(this->get_logger(), "    w: %.3f", qw);
        RCLCPP_INFO(this->get_logger(), "  orientation_name: \"%s\"", 
                    global_best.orientation_name.c_str());
        
        // 输出前5个最低点
        if (all_results.size() > 1)
        {
          std::sort(all_results.begin(), all_results.end(),
                   [](const TestResult& a, const TestResult& b) {
                     return a.z < b.z;
                   });
          
          RCLCPP_INFO(this->get_logger(), "");
          RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
          RCLCPP_INFO(this->get_logger(), "📋 前 10 个最低点:");
          RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
          RCLCPP_INFO(this->get_logger(), "");
          
          int show_count = std::min(10, (int)all_results.size());
          for (int i = 0; i < show_count; ++i)
          {
            const auto& r = all_results[i];
            RCLCPP_INFO(this->get_logger(), 
                       "   %2d. Z=%.4f @ (X=%.2f, Y=%.2f) [%s]",
                       i+1, r.z, r.x, r.y, r.orientation_name.c_str());
          }
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "❌ 在整个搜索空间中未找到任何可达位置!");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "💡 建议:");
        RCLCPP_INFO(this->get_logger(), "   1. 扩大 X、Y 搜索范围");
        RCLCPP_INFO(this->get_logger(), "   2. 增大起始 Z 高度 (z_start)");
        RCLCPP_INFO(this->get_logger(), "   3. 减小步长以获得更精细的搜索");
        RCLCPP_INFO(this->get_logger(), "   4. 检查机器人工作空间限制");
        RCLCPP_INFO(this->get_logger(), "   5. 在 RViz 中可视化机器人模型");
      }
      
      RCLCPP_INFO(this->get_logger(), "");
      RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════╝");
      RCLCPP_INFO(this->get_logger(), "");
      
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
  auto node = std::make_shared<GlobalMinZFinder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}