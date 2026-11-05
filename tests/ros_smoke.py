#!/usr/bin/env python3
"""ROS integration checks: real node, topics, synthetic scans, wall-clock watchdog."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('avoidance_smoke_test', anonymous=True)
received = []
lock = threading.Lock()


def output(msg):
    with lock:
        received.append((time.monotonic(), msg.linear.x, msg.angular.z))


rospy.Subscriber('/cmd_vel', Twist, output, queue_size=100)
commands = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
scans = rospy.Publisher('/base_scan', LaserScan, queue_size=1)


def drive(duration, *, obstacle=False, scan=True, command=True, invalid=False):
    since = time.monotonic()
    while time.monotonic() - since < duration:
        if command:
            msg = Twist()
            msg.linear.x = .5
            commands.publish(msg)
        if scan:
            msg = LaserScan()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'base_link'
            msg.angle_min = -math.pi
            msg.angle_increment = 2 * math.pi / 360
            msg.angle_max = msg.angle_min + 359 * msg.angle_increment
            msg.range_min, msg.range_max = .02, 8.0
            msg.ranges = [float('inf')] * 360
            if obstacle:
                for i in range(360):
                    a = msg.angle_min + i * msg.angle_increment
                    projection = 1.2 * math.cos(a)
                    discriminant = .35**2 - (1.2**2 - projection**2)
                    if projection > 0 and discriminant >= 0:
                        msg.ranges[i] = projection - math.sqrt(discriminant)
            if invalid:
                msg.ranges[180] = float('nan')
            scans.publish(msg)
        time.sleep(.04)
    with lock:
        return [(v, w) for t, v, w in received if t > since + duration * .6]


def require(samples, check, label):
    assert samples, label + ': no node output'
    assert all(check(v, w) for v, w in samples), (label, samples)


def stopped(v, w):
    return abs(v) < 1e-8 and abs(w) < 1e-8


deadline = time.monotonic() + 10
while commands.get_num_connections() == 0 or scans.get_num_connections() == 0:
    assert time.monotonic() < deadline, 'Node did not subscribe'
    time.sleep(.05)
require(drive(.6, scan=False), stopped, 'No initial scan')
require(drive(.8), lambda v, w: abs(v-.5)<1e-8 and abs(w)<1e-8, 'Clear path passthrough')
require(drive(.8, obstacle=True), lambda v, w: v>0 and abs(w)>.1, 'Steer while moving')
require(drive(.8, invalid=True), stopped, 'Malformed scan')
require(drive(.8), lambda v, w: v>0, 'Recover with valid scan')
require(drive(1.2, command=False), stopped, 'Command timeout')
require(drive(.8), lambda v, w: v>0, 'Recover with new command')
require(drive(1.0, scan=False), stopped, 'Scan timeout')
print('ROS smoke checks passed')
