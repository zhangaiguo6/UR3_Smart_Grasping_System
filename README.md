# UR3 智能柔顺抓取仿真系统

本项目为 UR3 机械臂与 Robotiq 夹爪在 Gazebo 中的柔顺抓取仿真大作业，主题为“基于状态机的异常容错控制”。

## 一、项目目标

本项目实现了 UR3 机械臂在 Gazebo 仿真环境下对桌面方块的柔顺抓取与拖动控制。系统包括底层力/位置混合控制脚本，以及上层状态机监督逻辑，用于检测 Gazebo、控制器、力传感器滤波话题等关键运行条件，并在异常情况下进行容错处理。

## 二、当前完成情况

目前已经完成以下内容：

* 成功编译 ROS Noetic + Gazebo 仿真工作空间。
* 成功启动 UR3 机械臂、Robotiq 夹爪、桌子和方块场景。
* 成功运行底层柔顺抓取脚本 `compliance_controller_example.py`。
* 成功启动并使用 `/wrench/filtered` 力传感器滤波话题。
* 成功实现上层状态机脚本 `smart_grasp_state_machine.py`。
* 成功保存位置跟踪曲线、力跟踪曲线、状态机日志和 ROS 探针结果。
* 已通过 GitHub 进行版本管理。

## 三、系统核心结构

项目主要文件如下：

* `scripts/compliance_controller_example.py`：底层柔顺抓取与力/位置混合控制脚本。
* `scripts/smart_grasp_state_machine.py`：上层状态机异常容错控制脚本。
* `results/task_plots/`：实验生成的位置与力跟踪曲线。
* `results/logs/`：状态机运行日志。
* `results/ros_probe/`：ROS 控制器、话题、服务和关节名称探针结果。

## 四、真实 ROS 控制器与关节名称

通过 `rosservice` 和 `rostopic` 探针确认，本仿真环境中的关键名称如下：

* UR3 机械臂轨迹控制器：`scaled_pos_joint_traj_controller`
* Robotiq 夹爪控制器：`gripper_controller`
* 夹爪主关节：`finger_joint`
* 力传感器原始话题：`/wrench`
* 力传感器滤波话题：`/wrench/filtered`

## 五、运行方法

### 1. 启动 Gazebo 仿真环境

```bash
source /opt/ros/noetic/setup.bash
source ~/ur3_sim/devel/setup.bash
roslaunch ur_gripper_gazebo ur3_cubes_example.launch grasp_plugin:=1
```

### 2. 启动力传感器滤波器

另开一个终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ur3_sim/devel/setup.bash
rosrun ur_control ft_filter.py -t wrench -ot wrench/filtered -z
```

### 3. 运行状态机控制脚本

再开一个终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ur3_sim/devel/setup.bash
rosrun ur_control smart_grasp_state_machine.py _table_side:=0.02 _table_duration:=40 _target_force_z:=5 _max_retries:=1
```

## 六、状态机设计

状态机主要包含以下状态：

* `INIT`：系统初始化。
* `CHECK_GAZEBO`：检查 Gazebo 世界和模型是否正常。
* `CHECK_CONTROLLERS`：检查机械臂和夹爪控制器是否处于 running 状态。
* `CHECK_WRENCH`：检查 `/wrench/filtered` 是否存在，必要时自动启动滤波器。
* `RUN_GRASP_TASK`：调用底层柔顺抓取任务。
* `VERIFY_RESULT`：检查实验曲线和结果文件是否生成。
* `ERROR_RECOVERY`：任务失败后使用更保守参数重试。
* `DONE`：任务成功完成。
* `FAILED`：任务失败退出。

## 七、实验结果

实验中，机械臂成功完成了以下流程：

1. 打开夹爪。
2. 移动到方块附近。
3. 对准并夹取方块。
4. 在桌面上执行 XY 平面方形拖动。
5. 在 Z 方向进行恒力柔顺控制。
6. 保存位置跟踪曲线和力跟踪曲线。

实验曲线保存在：

* `results/task_plots/table_slide_865_position.png`
* `results/task_plots/table_slide_865_force.png`

状态机成功日志保存在：

* `results/logs/state_machine_success_2.log`

## 八、已知问题与改进方向

当前系统已实现基础柔顺抓取与状态机容错控制，但仍有进一步优化空间：

* Gazebo 力控过程中 Z 方向存在一定周期性振荡。
* 重复实验前需要重启 Gazebo 或复位方块位置，否则可能出现 IK 求解失败。
* 后续可加入 `RESET_WORLD` 状态，实现自动调用 `/gazebo/reset_world`。
* 后续可加入抓取成功检测，例如检查方块是否随夹爪移动。
* 后续可加入力异常检测，例如 Fz 长时间偏离目标值时触发恢复策略。

## 九、项目总结

本项目完成了 UR3 机械臂在 Gazebo 中的柔顺抓取仿真，并在底层力/位置混合控制基础上加入了上层状态机监督逻辑。系统能够自动检查 Gazebo、控制器和力传感器滤波话题，并在任务失败时进行容错重试，满足“基于状态机的异常容错控制”大作业要求。

