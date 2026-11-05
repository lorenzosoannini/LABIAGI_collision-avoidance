#!/usr/bin/env bash
# Run inside Xvfb (or an existing X11 session). Captures the actual Stage window.
set -euo pipefail
output_dir="${1:-/tmp}"
mkdir -p "$output_dir"
roslaunch collision_avoidance stage_demo.launch gui:=true > "$output_dir/stage.log" 2>&1 &
launch_pid=$!
capture_pid=""
cleanup() {
  if [[ -n "$capture_pid" ]]; then
    kill -INT "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
  fi
  kill "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  cat "$output_dir/stage.log"
}
trap cleanup EXIT
# A scan confirms Stage's GUI/event loop and the ROS clock are running.
python3 - <<'PY'
import rospy
from sensor_msgs.msg import LaserScan
rospy.init_node('stage_recording_ready', anonymous=True)
rospy.wait_for_message('/base_scan', LaserScan, timeout=20)
PY
ffmpeg -y -loglevel warning -f x11grab -framerate 12 -video_size 1000x650 \
  -i "$DISPLAY+0,0" -c:v libx264 -preset ultrafast -crf 22 -pix_fmt yuv420p \
  "$output_dir/stage-demo.mp4" > "$output_dir/stage-recording.log" 2>&1 &
capture_pid=$!
# Let the recorder initialize while the controller holds the stationary robot.
sleep 0.5
trial_status=0
python3 tools/stage_trial.py --output "$output_dir/stage-trajectory.csv" || trial_status=$?
sleep 1
kill -INT "$capture_pid" 2>/dev/null || true
wait "$capture_pid"
capture_pid=""
# Only resizing, frame sampling and palette conversion; no reconstructed scene.
ffmpeg -y -loglevel warning -i "$output_dir/stage-demo.mp4" \
  -filter_complex '[0:v]fps=10,scale=900:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3' \
  -loop 0 "$output_dir/stage-demo.gif"
ffmpeg -y -loglevel warning -ss 6 -i "$output_dir/stage-demo.mp4" \
  -frames:v 1 "$output_dir/stage-demo-preview.png"
exit "$trial_status"
