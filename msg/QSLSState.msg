# 无人机吊载系统状态消息
geometry_msgs/Vector3 pL       # 负载位置 (m)
geometry_msgs/Vector3 vL     # 负载速度 (m/s)
geometry_msgs/Vector3 pQ     # 无人机位置 (m)
geometry_msgs/Vector3 vQ     # 无人机速度 (m/s)
geometry_msgs/Vector3 q      # 负载方向
geometry_msgs/Vector3 w       # 负载角速度
geometry_msgs/Quaternion quat  # 无人机姿态四元数
geometry_msgs/Vector3 bL     # 负载Disturbance
geometry_msgs/Vector3 bQ     # 无人机Disturbance
