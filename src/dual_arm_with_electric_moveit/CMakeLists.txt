cmake_minimum_required(VERSION 3.22)
project(dual_arm_with_electric_moveit)

# 添加 C++17 标准支持
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== 查找依赖包 ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ========== 第一步：生成服务接口 ==========
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveArm.srv"
  "srv/MoveArmCartesian.srv"
  DEPENDENCIES geometry_msgs
)

# ========== 第二步：获取类型支持目标 ==========
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ========== 第三步：编译可执行文件 ==========

# 1. 原始服务节点（基础版本）
add_executable(arm_move_service src/arm_move_service.cpp)

ament_target_dependencies(arm_move_service
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

target_link_libraries(arm_move_service "${cpp_typesupport_target}")

# 2. 带约束的服务节点（姿态约束 + 笛卡尔路径）
add_executable(arm_move_service_constraints src/arm_move_service.cpp)

ament_target_dependencies(arm_move_service_constraints
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
)

target_link_libraries(arm_move_service_constraints "${cpp_typesupport_target}")

# 3. IKFast版本服务节点
add_executable(arm_move_service_ikfast src/arm_move_service_ikfast.cpp)

ament_target_dependencies(arm_move_service_ikfast
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning  # 这个包已经包含了 robot_model_loader
  moveit_kinematics
  moveit_msgs
  geometry_msgs
)

target_link_libraries(arm_move_service_ikfast "${cpp_typesupport_target}")

# 4. 原始测试程序（可选）
add_executable(move_right_arm_node src/move_right_arm.cpp)

ament_target_dependencies(move_right_arm_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# ========== 第四步：安装 ==========
# 安装可执行文件
install(TARGETS
  arm_move_service
  arm_move_service_constraints
  arm_move_service_ikfast
  move_right_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

# 安装配置文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config 
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安装setup_assistant配置
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(FILES .setup_assistant 
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ========== 测试配置（可选） ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()