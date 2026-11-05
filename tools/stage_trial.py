#!/usr/bin/env python3
"""Drive the real ROS+Stage stack toward a goal and verify the observed trajectory."""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path('/tmp/stage-trajectory.csv'))
parser.add_argument('--timeout', type=float, default=60.0, help='Wall-clock deadline in seconds')
args = parser.parse_args(rospy.myargv()[1:])
rospy.init_node('stage_avoidance_trial', anonymous=True)
lock = threading.Lock()
state = {'pose': None, 'output': (0., 0.), 'scan': 0., 'odom_time': 0., 'output_time': 0., 'minimum_gap': float('inf'), 'maximum_y': 0., 'pose_samples': 0,
         'first_sim_time': None, 'last_sim_time': None}


def odometry(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    with lock:
        state['pose'] = (p.x, p.y, yaw)
        state['odom_time'] = time.monotonic()
        gap = math.hypot(max(abs(p.x-2.5)-.45, 0), max(abs(p.y)-.45, 0))-.20
        state['minimum_gap'] = min(state['minimum_gap'], gap)
        state['maximum_y'] = max(state['maximum_y'], abs(p.y))
        state['pose_samples'] += 1
        stamp = msg.header.stamp.to_sec()
        if state['first_sim_time'] is None:
            state['first_sim_time'] = stamp
        state['last_sim_time'] = stamp


def output(msg):
    with lock:
        state['output'] = (msg.linear.x, msg.angular.z)
        state['output_time'] = time.monotonic()


def scan(msg):
    with lock:
        state['scan'] = time.monotonic()


# Ground truth is used for verification and the *upstream* goal driver only.
# The avoidance node subscribes solely to the laser and raw desired velocity.
rospy.Subscriber('/base_pose_ground_truth', Odometry, odometry, queue_size=1)
rospy.Subscriber('/cmd_vel', Twist, output, queue_size=1)
rospy.Subscriber('/base_scan', LaserScan, scan, queue_size=1)
publisher = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
start = time.monotonic()
rows = []
minimum = float('inf')
max_deviation = 0.
moving_turns = 0
reached = False
try:
    while not rospy.is_shutdown() and time.monotonic()-start < args.timeout:
        with lock:
            current = state.copy()
        now = time.monotonic()
        pose = current['pose']
        if pose is None or current['output_time'] == 0 or now-current['scan'] > 1.0 or publisher.get_num_connections() == 0:
            time.sleep(.05)
            continue
        if now-current['odom_time'] > .5:
            raise RuntimeError('Stage ground truth has stopped updating')
        x, y, yaw = pose
        distance = math.hypot(6.5-x, y)
        if distance < .15:
            reached = True
            break
        # Exact distance from robot center to the axis-aligned Stage obstacle box.
        gap = math.hypot(max(abs(x-2.5)-.45, 0), max(abs(y)-.45, 0))-.20
        minimum = min(minimum, gap)
        max_deviation = max(max_deviation, abs(y))
        if current['minimum_gap'] < .095:
            raise RuntimeError(f'Clearance violation: {gap:.3f} m')
        desired = Twist()
        desired.linear.x = min(.55, distance)
        error = math.atan2(-y, 6.5-x)-yaw
        error = math.atan2(math.sin(error), math.cos(error))
        desired.angular.z = max(-1.2, min(1.2, 1.6*error))
        publisher.publish(desired)
        v, w = current['output']
        if now-current['output_time'] > .5:
            raise RuntimeError('Avoidance output is missing or stale')
        if not (math.isfinite(v) and math.isfinite(w) and 0 <= v <= .600001 and abs(w) <= 1.500001):
            raise RuntimeError('Invalid/out-of-bounds output command')
        moving_turns += v > .05 and abs(w-desired.angular.z) > .15
        rows.append([rospy.Time.now().to_sec(), x, y, yaw, desired.linear.x, desired.angular.z, v, w, gap])
        time.sleep(.05)
finally:
    # Give the node time to deliver a stop before ending the trial.
    for _ in range(5):
        publisher.publish(Twist())
        time.sleep(.05)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t', 'x', 'y', 'yaw', 'desired_v', 'desired_w', 'v', 'w', 'clearance'])
        writer.writerows(rows)
with lock:
    final = state.copy()
simulated = (final['last_sim_time'] or 0)-(final['first_sim_time'] or 0)
wall_elapsed = time.monotonic()-start
report = {'reached_goal': reached, 'samples': len(rows), 'ground_truth_samples': final['pose_samples'],
          'simulated_seconds': simulated, 'wall_seconds': wall_elapsed,
          'real_time_factor': simulated/wall_elapsed,
          'min_surface_clearance_m': final['minimum_gap'], 'max_lateral_deviation_m': final['maximum_y'],
          'moving_avoidance_samples': moving_turns}
args.output.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
if not reached or moving_turns < 3 or max_deviation < .4 or final['minimum_gap'] < .095 or not (.5 < simulated/wall_elapsed < 1.5):
    raise SystemExit('Stage trial failed: goal arrival and a moving bypass are required')
