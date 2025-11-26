cmake_minimum_required(VERSION 3.22)
project(dual_arm_agv_moveit)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveArm.srv"
  DEPENDENCIES geometry_msgs
)

# 编译arm_move_service节点
add_executable(arm_move_service src/arm_move_service.cpp)

# 设置arm_move_service依赖关系
ament_target_dependencies(arm_move_service
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
)

# 确保服务接口先生成
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(arm_move_service "${cpp_typesupport_target}")

# ========== 新增工具 ==========
# 编译 find_min_z_position 节点
add_executable(find_min_z_position src/find_min_z_position.cpp)

ament_target_dependencies(find_min_z_position
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
)

# 编译 planning_diagnostic 节点
add_executable(planning_diagnostic src/planning_diagnostic.cpp)

ament_target_dependencies(planning_diagnostic
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
)

# 安装可执行文件
install(TARGETS
  arm_move_service
  find_min_z_position
  planning_diagnostic
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# 安装配置文件
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 安装服务定义文件
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()