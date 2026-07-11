#!/bin/bash
# Guardian Stride — Full System Startup Script
# Run on DK-2500: bash startup.sh

echo 0 | sudo -S ip link set can0 type can bitrate 1000000 restart-ms 100 2>/dev/null
echo 0 | sudo -S ip link set up can0 2>/dev/null
echo "CAN up: $(ip link show can0 | head -1)"

# Kill old processes
pkill -f "ros2 run" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
sleep 1

SESSION=guardian
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n camera
tmux set-option -t $SESSION mouse on

S="source /opt/ros/humble/setup.bash && source /home/guardian/Desktop/ros2_ws/install/setup.bash 2>/dev/null"

# 1. RealSense Camera
tmux send-keys -t $SESSION:camera "bash -c '$S && ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true pointcloud.enable:=true align_depth.enable:=true color_fps:=30 depth_fps:=30'" Enter
echo "[1/10] Camera launching..."
sleep 2

# 2. Web Video Server
tmux new-window -t $SESSION -n webvideo
tmux send-keys -t $SESSION:webvideo "bash -c '$S && ros2 run web_video_server web_video_server --ros-args -p port:=8080'" Enter
echo "[2/10] Web video server"

# 3. Rosbridge
tmux new-window -t $SESSION -n rosbridge
tmux send-keys -t $SESSION:rosbridge "bash -c '$S && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090'" Enter
echo "[3/10] Rosbridge"

# 4. Voice ASR (BOYA mic)
tmux new-window -t $SESSION -n asr
tmux send-keys -t $SESSION:asr "bash -c '$S && ros2 run guardian_voice voice_asr_node --ros-args -p alsa_device:=plughw:1,0 -p energy_threshold:=0.010 -p asr_model:=Systran/faster-whisper-small -p tts_mute_duration_s:=6.0'" Enter
echo "[4/10] ASR voice"

# 5. LLM Assistant
tmux new-window -t $SESSION -n llm
tmux send-keys -t $SESSION:llm "bash -c '$S && ros2 run guardian_voice voice_assistant_llm'" Enter
echo "[5/10] LLM assistant"

# 6. TTS
tmux new-window -t $SESSION -n tts
tmux send-keys -t $SESSION:tts "bash -c '$S && ros2 run guardian_voice tts_node'" Enter
echo "[6/10] TTS"

# 7. Depth Obstacle
tmux new-window -t $SESSION -n depth
tmux send-keys -t $SESSION:depth "bash -c '$S && ros2 run guardian_voice depth_obstacle_node'" Enter
echo "[7/10] Depth obstacle"

# 8. Depth Viz
tmux new-window -t $SESSION -n depthviz
tmux send-keys -t $SESSION:depthviz "bash -c '$S && ros2 run guardian_voice depth_viz_node'" Enter
echo "[8/10] Depth viz"

# 9. System Monitor
tmux new-window -t $SESSION -n sysmon
tmux send-keys -t $SESSION:sysmon "bash -c '$S && ros2 run guardian_voice sys_monitor_node'" Enter
echo "[9/10] System monitor"

# 10. Mapping + Traversability (replaces semantic segmentation)
tmux new-window -t $SESSION -n mapping
tmux send-keys -t $SESSION:mapping "bash -c '$S && ros2 run guardian_voice mapping_node'" Enter
echo "[10/10] Mapping + traversability"

echo "=== ALL LAUNCHED ==="
sleep 6
tmux list-windows -t $SESSION
