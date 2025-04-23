话题订阅：
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
        - 旋量（twist）

输入文件：
1. 建图文件
    - 格式： map.bin
    - 内容：
        1. arrow points
        2. dash points
        3. all corner points
        4. slot index