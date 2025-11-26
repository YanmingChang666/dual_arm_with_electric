# 通用 Z 轴最低点搜索工具

## 功能说明

这个工具可以在**任意 X、Y 位置**和**任意姿态(orientation)**下搜索末端执行器的 Z 轴最低可达位置。

## 编译

```bash
cd ~/your_workspace
colcon build --packages-select dual_arm_agv_moveit
source install/setup.bash
```

## 使用方法

### 方法 1: 使用四元数指定姿态(推荐)

```bash
ros2 run dual_arm_agv_moveit find_min_z_universal \
  --ros-args \
  -p arm_group:=arm_l \
  -p test_x:=0.26087 \
  -p test_y:=-0.87515 \
  -p start_z:=1.5 \
  -p min_z:=0.3 \
  -p z_step:=0.02 \
  -p use_quaternion:=true \
  -p quat_x:=0.5 \
  -p quat_y:=-0.5 \
  -p quat_z:=-0.5 \
  -p quat_w:=0.5 \
  -p verbose:=true
```

### 方法 2: 使用欧拉角指定姿态

```bash
ros2 run dual_arm_agv_moveit find_min_z_universal \
  --ros-args \
  -p arm_group:=arm_l \
  -p test_x:=0.3 \
  -p test_y:=-0.8 \
  -p start_z:=1.5 \
  -p min_z:=0.3 \
  -p z_step:=0.02 \
  -p use_quaternion:=false \
  -p roll_deg:=0.0 \
  -p pitch_deg:=90.0 \
  -p yaw_deg:=0.0 \
  -p verbose:=true
```

### 方法 3: 测试右臂

```bash
ros2 run dual_arm_agv_moveit find_min_z_universal \
  --ros-args \
  -p arm_group:=arm_r \
  -p test_x:=0.26087 \
  -p test_y:=0.87515 \
  -p start_z:=1.5 \
  -p min_z:=0.3 \
  -p z_step:=0.03 \
  -p use_quaternion:=true \
  -p quat_x:=0.5 \
  -p quat_y:=0.5 \
  -p quat_z:=0.5 \
  -p quat_w:=0.5 \
  -p verbose:=false
```

## 参数说明

### 必需参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `arm_group` | string | arm_l | 规划组名称 (arm_l 或 arm_r) |
| `test_x` | double | 0.26087 | 测试的 X 坐标 (米) |
| `test_y` | double | -0.87515 | 测试的 Y 坐标 (米) |
| `start_z` | double | 1.5 | 搜索起始高度 (米) |
| `min_z` | double | 0.3 | 搜索最低高度 (米) |
| `z_step` | double | 0.02 | Z 轴搜索步长 (米) |

### 姿态参数

#### 使用四元数 (use_quaternion:=true)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_quaternion` | bool | true | 是否使用四元数 |
| `quat_x` | double | 0.5 | 四元数 X 分量 |
| `quat_y` | double | -0.5 | 四元数 Y 分量 |
| `quat_z` | double | -0.5 | 四元数 Z 分量 |
| `quat_w` | double | 0.5 | 四元数 W 分量 |

#### 使用欧拉角 (use_quaternion:=false)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_quaternion` | bool | true | 设为 false 使用欧拉角 |
| `roll_deg` | double | 0.0 | 绕 X 轴旋转角度 (度) |
| `pitch_deg` | double | 0.0 | 绕 Y 轴旋转角度 (度) |
| `yaw_deg` | double | 0.0 | 绕 Z 轴旋转角度 (度) |

### 规划参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `planning_time` | double | 5.0 | 每次规划的最大时间 (秒) |
| `planning_attempts` | int | 10 | 每次规划的尝试次数 |
| `consecutive_fail_threshold` | int | 3 | 连续失败多少次后停止 |

### 输出参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `verbose` | bool | true | 是否详细输出每次测试结果 |

## 常见姿态配置

### 1. 垂直向下 (常用于抓取)

**四元数方式:**
```bash
-p quat_x:=0.5 -p quat_y:=-0.5 -p quat_z:=-0.5 -p quat_w:=0.5
```

**欧拉角方式:**
```bash
-p use_quaternion:=false -p roll_deg:=0 -p pitch_deg:=0 -p yaw_deg:=0
```

### 2. 水平向前

**四元数方式:**
```bash
-p quat_x:=0.0 -p quat_y:=0.0 -p quat_z:=0.0 -p quat_w:=1.0
```

**欧拉角方式:**
```bash
-p use_quaternion:=false -p roll_deg:=0 -p pitch_deg:=90 -p yaw_deg:=0
```

### 3. 45度倾斜

**四元数方式:**
```bash
-p quat_x:=0.383 -p quat_y:=-0.383 -p quat_z:=-0.383 -p quat_w:=0.707
```

**欧拉角方式:**
```bash
-p use_quaternion:=false -p roll_deg:=0 -p pitch_deg:=45 -p yaw_deg:=0
```

### 4. 侧向抓取

**欧拉角方式:**
```bash
-p use_quaternion:=false -p roll_deg:=90 -p pitch_deg:=0 -p yaw_deg:=0
```

## 输出结果说明

程序会输出以下信息:

1. **当前状态**: 机械臂当前末端位置
2. **测试配置**: 搜索的位置和姿态
3. **搜索过程**: 每个 Z 值的测试结果
4. **统计结果**: 
   - 可达位置数量
   - 最高/最低可达 Z 值
   - 可达范围
   - 可达位置列表
5. **ROS2 服务调用命令**: 可直接复制使用
6. **YAML 配置**: 可保存到配置文件

## 使用技巧

### 1. 快速粗搜索
```bash
# 使用较大步长快速找到大概范围
-p z_step:=0.05 -p verbose:=false
```

### 2. 精细搜索
```bash
# 在找到的范围内使用小步长精确搜索
-p start_z:=1.2 -p min_z:=1.0 -p z_step:=0.005 -p verbose:=true
```

### 3. 多位置扫描

创建一个脚本批量测试多个位置:

```bash
#!/bin/bash

# 测试多个 X 位置
for x in 0.2 0.25 0.3 0.35; do
  echo "Testing X=$x"
  ros2 run dual_arm_agv_moveit find_min_z_universal \
    --ros-args \
    -p test_x:=$x \
    -p test_y:=-0.8 \
    -p verbose:=false
done
```

### 4. 多姿态扫描

测试不同角度:

```bash
#!/bin/bash

# 测试不同俯仰角
for pitch in 0 15 30 45 60 75 90; do
  echo "Testing Pitch=$pitch°"
  ros2 run dual_arm_agv_moveit find_min_z_universal \
    --ros-args \
    -p use_quaternion:=false \
    -p pitch_deg:=$pitch \
    -p verbose:=false
done
```

## 故障排查

### 问题 1: 所有位置都不可达

**可能原因:**
- 起始 Z 值太低
- X、Y 位置超出工作空间
- 姿态不合理

**解决方法:**
```bash
# 1. 增大起始高度
-p start_z:=2.0

# 2. 调整位置到工作空间中心
-p test_x:=0.3 -p test_y:=-0.7

# 3. 使用默认姿态
-p use_quaternion:=true -p quat_x:=0.5 -p quat_y:=-0.5 -p quat_z:=-0.5 -p quat_w:=0.5
```

### 问题 2: 搜索太慢

**解决方法:**
```bash
# 减少规划时间和尝试次数
-p planning_time:=3.0 -p planning_attempts:=5

# 增大步长
-p z_step:=0.05

# 关闭详细输出
-p verbose:=false
```

### 问题 3: 连续失败阈值太敏感

**解决方法:**
```bash
# 增加连续失败阈值
-p consecutive_fail_threshold:=5
```

## 与其他工具的区别

| 特性 | find_min_z_position | find_min_z_universal |
|------|---------------------|----------------------|
| X、Y 位置 | 固定参数 | 可自由指定 |
| 姿态设置 | 固定四元数 | 支持四元数和欧拉角 |
| 输出格式 | 简单 | 详细统计 + 格式化 |
| 姿态显示 | 仅四元数 | 四元数和欧拉角都显示 |
| 适用场景 | 快速测试特定位置 | 通用工作空间分析 |

## 实际应用示例

### 场景 1: 电池拆解 - 寻找最低抓取点

```bash
ros2 run dual_arm_agv_moveit find_min_z_universal \
  --ros-args \
  -p arm_group:=arm_l \
  -p test_x:=0.4 \
  -p test_y:=-0.6 \
  -p start_z:=1.2 \
  -p min_z:=0.4 \
  -p z_step:=0.01 \
  -p use_quaternion:=true \
  -p quat_x:=0.5 -p quat_y:=-0.5 -p quat_z:=-0.5 -p quat_w:=0.5
```

### 场景 2: 装配任务 - 测试水平插入

```bash
ros2 run dual_arm_agv_moveit find_min_z_universal \
  --ros-args \
  -p arm_group:=arm_r \
  -p test_x:=0.35 \
  -p test_y:=0.7 \
  -p start_z:=1.0 \
  -p min_z:=0.5 \
  -p z_step:=0.02 \
  -p use_quaternion:=false \
  -p roll_deg:=0 -p pitch_deg:=90 -p yaw_deg:=45
```

### 场景 3: 视觉检测 - 找最佳相机高度

```bash
ros2 run dual_arm_agv_moveit find_min_z_universal \
  --ros-args \
  -p arm_group:=arm_l \
  -p test_x:=0.5 \
  -p test_y:=0.0 \
  -p start_z:=1.5 \
  -p min_z:=0.8 \
  -p z_step:=0.03 \
  -p use_quaternion:=false \
  -p roll_deg:=0 -p pitch_deg:=-30 -p yaw_deg:=0
```
