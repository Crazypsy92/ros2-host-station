# 上位机与下位机/仿真节点系统状态
std_msgs/Header header

string state             # INIT, READY, RUNNING, PAUSED, ERROR, EMERGENCY_STOP
bool emergency_stop      # 急停标志
float32 loop_frequency   # 运行频率 (Hz)
uint32 packets_tx        # 发送数据包计数
uint32 packets_rx        # 接收数据包计数
uint32 error_count       # 通信/硬件错误计数
string active_backend    # 当前后端: SIMULATOR, MUJOCO, MCU_SERIAL, MCU_UDP
string status_message    # 状态提示或报警信息
