# dual_arm_with_electric


ros2 run dual_arm_agv_moveit arm_move_service 

#绕 X 轴旋转 90∘
ros2 service call /move_arm dual_arm_agv_moveit/srv/MoveArm "{
  control_mode: 'arm_l',
  target_pose_left: {
    position: {x: 0.26087, y: -0.87515, z: 1.31267},
    orientation: {x: 0.7071, y: 0.0, z: 0.0, w: 0.7071}
  }
}"

#挠头
ros2 service call /move_arm dual_arm_agv_moveit/srv/MoveArm "{
  control_mode: 'arm_l',
  target_pose_left: {
    position: {x: 0.0, y: 0.0, z: 2.18},
    orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}
  }
}"

ros2 service call /move_arm dual_arm_agv_moveit/srv/MoveArm "{
  control_mode: 'arm_r',
  target_pose_right: {
    position: {x: 0.0, y: 0.0, z: 2.18},
    orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}
  }
}"

#测试z方向最低点位置（确定x,y坐标以及旋转方向）：
ros2 run dual_arm_agv_moveit find_min_z_position   --ros-args   -p arm_group:=arm_l   -p base_x:=0.26087   -p base_y:=-0.87515   -p start_z:=1.2   -p min_z:=0.5   -p z_step:=0.05 -p orientation_x:=0.0 -p orientation_y:=0.0 -p orientation_z:=0.0 -p orientation_w:=1.0


