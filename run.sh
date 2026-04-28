#配置环境
# chomd +x ./tools/setup_vrx_deps.sh
# bash ./tools/setup_vrx_deps.sh

# 编译
colcon build --merge-install
source install/setup.bash

# 运行
# 先生成wamv模型
mkdir -p $(ros2 pkg prefix vrx_gazebo)/share/vrx_gazebo/models/wamv/tmp
ros2 launch vrx_gazebo generate_wamv.launch.py \
  wamv_target:=$(ros2 pkg prefix vrx_gazebo)/share/vrx_gazebo/models/wamv/tmp/model.urdf
# 启动gazebo
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta


# 开发
# 编译包：避障，键盘控制，nav2
source install/setup.bash && colcon build --merge-install --packages-select vrx_tutorial

# 键盘控制（WASD控制运动）
ros2 run vrx_tutorial keyboard_teleop
# 更快
ros2 run vrx_tutorial keyboard_teleop --ros-args -p thrust_step:=100.0 -p max_thrust:=1500.0


# 速度转换推力
ros2 run vrx_tutorial cmd_vel_to_wamv
# 终端2
# 让船直行（linear.x = 1.0）
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' --rate 10
# 转向（angular.z = 0.5，向左转）
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.5}}' --rate 10


# 自动避障
# 需要先启动gazebo仿真环境
ros2 launch vrx_tutorial nav2.launch.py
# ros2 run tf2_tools view_frames #画TF树
# 然后打开rviz2
#   Fixed Frame： 修改为map（注意，修改为map之后，需要手动缩放找找地图在哪）
# 依次添加：
#   1. TF： 检查基本的transform都有没有
#   2. Map： global map（/global_costmap/costmap）
#   3. Map： local map（/local_costmap/costmap）
#   4. LaserScan： 激光（/wamv/sensors/lidars/lidar_wamv_sensor/scan）
#   5. Path： 规划路径
#   6. PointCloud2： 点云（/wamv/sensors/lidars/lidar_wamv_sensor/points）
# 之后在rviz2中手动点击goal point，就能看到规划结果了，USV自动航行。
rviz2 -d rviz/tutorial.rviz

# 采用launch脚本一次性启动：gazebo、nav2、rviz2
######################################################
# 启动顺序：先起 gazebo（含 WAM-V 与 ros_gz bridges），5 秒后起 nav2，3 秒后起 rviz2，避免 nav2 节点没等到 /clock 就 lifecycle 卡死。

# 可选参数：

# world:=sydney_regatta 切换世界
# rviz_config:=/绝对/路径/xxx.rviz 换配置（默认就是 vrx_tutorial/rviz/tutorial.rviz）
# nav2_delay:=5.0 / rviz_delay:=3.0 调延迟
# use_sim_time:=true
# 注意：首次使用前 WAM-V 模型仍需生成（run.sh 里那两条 generate_wamv 命令），那是一次性的预处理，没放进这个 launch。
######################################################
source install/setup.bash && ros2 launch vrx_tutorial bringup.launch.py
