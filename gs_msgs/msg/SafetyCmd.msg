std_msgs/Header header
string state              # FSM 状态名
bool brake
float32 damp_level        # 0.0 ~ 1.0
string alert_text         # 注入 TTS 抢占队列（空字符串=无警报）
