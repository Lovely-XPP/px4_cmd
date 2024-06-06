std_msgs/Header header

## 载具
uint8 Vehicle
# 载具列举
uint8 Multicopter = 0 
uint8 FixWing = 1

## 控制命令模式 
uint8 Mode
# Mode 控制模式枚举
uint8 Idle = 0
uint8 Takeoff = 1
uint8 Move = 2
uint8 Hover = 3
uint8 Loiter = 3
uint8 Trajectory = 4
uint8 User_define = 5
uint8 Gliding = 6       ### 固定翼滑行模式

## 移动命令坐标系
uint8 Move_frame
## 坐标系枚举
uint8 ENU = 0           ### 惯性系
uint8 BODY = 1          ### 机体系
uint8 GLOBAL = 2        ### 地球系

## 移动命令下的子模式
uint8 Move_mode
## 移动命令下的子模式枚举
uint8 XYZ_POS = 0        ### 定点控制(绝对位置)
uint8 XY_VEL_Z_POS = 1   ### 定高速度控制 - 仅旋翼
uint8 XYZ_VEL = 2        ### 速度控制 - 仅旋翼
uint8 XYZ_REL_POS = 3    ### 定点控制(相对位置)
uint8 Custom_Command = 4 ### 自定义指令

# 自定义指令模式定义
uint8 Custom_Command_Mode
## 自定义指令下的模式枚举
uint8 Custom_Command_TargetLocal = 1        ### 局部绝对坐标模式
uint8 Custom_Command_TargetGlobal = 2       ### 地球绝对坐标模式
uint8 Custom_Command_TargetAttitude = 3     ### 推力姿态模式
# 自定义指令typemask
uint8 fx_custom_mode  ### 固定翼位置控制模式
uint8 fx_custom_mode_GlidingMode = 1   ### Gliding Mode
uint8 fx_custom_mode_TakeoffMode = 2   ### Takeoff Mode
uint8 fx_custom_mode_LandMode = 3      ### Land Mode
uint8 fx_custom_mode_LoiterMode = 4    ### Loiter Mode (fly a circle centred on setpoint).
uint8 fx_custom_mode_IdleMode = 5      ### Idle Mode (zero throttle, zero roll / pitch).
# custom command
float32[20] custom_cmd     ### custom command - [position] 3 [velocity] 3 [accelerate] 3 [Forceflag Yaw YawRate] 3 [Attitude (quaternion) {xyzw}] 4 [BodyRate] 3 [Thrust] 1

## 指令输入
float32[3] desire_cmd      ### desire command
float32 yaw_cmd            ### [rad]
float32 yaw_rate_cmd       ### [rad/s]
float32 ext_total_time     ### 外部指令运行总时间 [s]
float32 ext_time           ### 外部指令当前运行时间 [s]
