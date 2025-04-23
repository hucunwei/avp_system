planning功能包
- 实现：
    - 路径规划功能

- 输入文件：
    1. 建图文件
        - 格式： map.bin
        - 内容：
            1. arrow points
            2. dash points
            3. all corner points
            4. slot index

- 订阅话题：
    1. 2D Nav Goal 目标车位点
        - 话题： "/move_base_simple/goal"
        - 消息类型：geometry_msgs::PoseStamped
    2. 实时语义 定位信息（from 定位模块）
        - 话题： "/localization/out_pose"
        - 消息类型：nav_msgs::Odometry
            - 位姿（position + orientation）作为 小车的 初始位姿
            - 旋量（twist）无
    3. 实时真实 定位信息（from 仿真器）
        - 话题： "/gps_odom"
        - 消息类型：nav_msgs::Odometry
            - 位姿（position + orientation）作为 小车的 初始位姿
            - 旋量（twist）无
- 发布话题：
    1. 导航路径
        - 话题："/planning_node/planned_hybridA_path" 
        - 消息类型：nav_msgs/Path

- 具体功能： 
    - 从 定位信息 获得 小车 初始位置，订阅一次 话题（/localization/out_pose） 就立即取消订阅
    - 在 rviz 文件，除了 出发点小车marker 外，增加了 目标点小车 marker.
    - 目标点小车 自动修正 位姿（如 修正 车头方向 和 车位内的中点） 
    - 添加 两个 预备点， 包括了 可视化 小车，位姿               
    - 考虑 多段式 导航                                                        --- 待开发
    - 参考costmap概念，考虑 obstable 优化, 使得 车不会 靠着 obstable 导航       --- 待开发
    - 车的参数调整(轴距，步长，最大转向角等)，配合control                        --- 待优化

- 如何运行 planning 模块
    0. 设置config文件: /planning/config/planning_config.yaml
    ```

    ```   

    1. 编译 planning 模块
    ```
    cd /avp_system         # cd 工作空间
    
    # 只编译 planning 模块/功能包  
    catkin_make -DCATKIN_WHITELIST_PACKAGES="planning"      # 指定需要编译的功能包, 这会跳过其他功能包的编译，只编译 planning
    catkin_make -DCATKIN_WHITELIST_PACKAGES=""              # 如果你想恢复到编译所有功能包的状态，可以运行以下命令清除白名单
    ```

    2. 运行 planning 模块
    ```
    roslaunch planning planning.launch
    ```
    或者
    ```
    # terminal 1
    rosrun rviz rviz -d .../planning/src/ros_viewer/planning.rviz 

    # terminal 2  
    rosrun planning planning_node
    ```
    
