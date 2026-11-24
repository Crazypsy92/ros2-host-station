# 上位机下发电机/关节控制指令
std_msgs/Header header

uint8 MODE_POSITION = 1
uint8 MODE_VELOCITY = 2
uint8 MODE_EFFORT   = 3
uint8 MODE_DISABLE  = 4

uint8 control_mode

string[] joint_names
float64[] positions      # 目标角度/位置 (rad 或 m)
float64[] velocities     # 目标速度 (rad/s 或 m/s)
float64[] efforts        # 目标力矩/力 (Nm 或 N)
