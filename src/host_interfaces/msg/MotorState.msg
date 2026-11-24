# 下位机/仿真器反馈电机状态
std_msgs/Header header

string[] joint_names
float64[] positions      # 当前位置 (rad 或 m)
float64[] velocities     # 当前速度 (rad/s 或 m/s)
float64[] efforts        # 当前力矩/电流 (Nm 或 A)
float32[] temperatures   # 驱动器/电机温度 (℃)
uint16[] fault_codes     # 故障码 (0=正常)
