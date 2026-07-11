#!/usr/bin/env python3
"""Launch ade20k_clean on remote DK-2500 via SSH."""
import paramiko, time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.88', username='guardian', password='0', timeout=10)

# Start ade20k_clean in a new tmux window
cmd = (
    "tmux new-window -t guardian -n seg ; "
    "tmux send-keys -t guardian:seg "
    "'source /opt/ros/humble/setup.bash && "
    "source /home/guardian/Desktop/ros2_ws/install/setup.bash 2>/dev/null && "
    "ros2 run guardian_voice ade20k_clean' Enter ; "
    "echo TMUX_DONE"
)
stdin, stdout, stderr = ssh.exec_command(cmd)
print(stdout.read().decode().strip())
print(stderr.read().decode().strip())

time.sleep(4)

# Verify
print('\n=== ade20k process ===')
stdin, stdout, stderr = ssh.exec_command('ps aux | grep ade20k | grep -v grep')
out = stdout.read().decode().strip()
print(out if out else 'Not found yet, waiting...')

# Check via ros2
print('\n=== ROS2 seg nodes ===')
stdin, stdout, stderr = ssh.exec_command(
    'source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -iE "ade|seg"'
)
print(stdout.read().decode().strip() or '(none)')

# Check topic
print('\n=== /segmentation/overlay publishers ===')
stdin, stdout, stderr = ssh.exec_command(
    'source /opt/ros/humble/setup.bash && ros2 topic info /segmentation/overlay 2>/dev/null'
)
print(stdout.read().decode().strip())

ssh.close()
