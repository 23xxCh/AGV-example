# ============================================================
# AGVStatus.msg - AGV车辆状态消息
# ============================================================
#
# 【这是什么】
# 每台AGV定期发布自己的状态信息，包括位置、速度、当前任务等。
# 调度系统和其他AGV订阅此消息来了解车队整体状况。
#
# 【技术要点】
# - 状态消息通常以固定频率发布（如10Hz），类似心跳
# - 多车系统中，每台车有唯一的agv_id，通过话题名区分
#   例如：/agv_001/status, /agv_002/status
# - 电池电量低于阈值时，调度系统应安排充电任务
# - 状态枚举值：0=空闲，1=执行中，2=充电中，3=故障，4=离线
#
# 【多车系统中的应用】
# - 调度器根据各车状态分配任务（只给空闲车分配）
# - 冲突检测需要知道各车当前位置和速度
# - 死锁检测需要知道各车正在走哪条路径
# - 可视化界面显示车队总览

std_msgs/Header header

# AGV唯一标识符，如 "agv_001"
string agv_id

# 当前位置（地图坐标系下的位姿）
geometry_msgs/PoseWithCovarianceStamped pose

# 当前线速度（米/秒）
float64 linear_velocity

# 当前角速度（弧度/秒）
float64 angular_velocity

# 当前电池电量百分比（0~100）
float64 battery_level

# 车辆状态（0=空闲，1=执行任务中，2=充电中，3=故障，4=离线）
uint8 STATUS_IDLE = 0
uint8 STATUS_EXECUTING = 1
uint8 STATUS_CHARGING = 2
uint8 STATUS_ERROR = 3
uint8 STATUS_OFFLINE = 4
uint8 status

# 当前正在执行的任务ID，空闲时为空字符串
string current_task_id
