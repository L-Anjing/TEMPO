"""Static registered-cloud fixture; exercises goal_pose and shutdown without any chassis commands."""
import argparse
import copy
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class StaticFixture(Node):
    def __init__(self, include_floor=False):
        super().__init__("static_navigation_regression")
        self.odom = self.create_publisher(Odometry, "/aft_mapped_to_init", 10)
        self.cloud = self.create_publisher(PointCloud2, "/cloud_registered_full", 1)
        self.goal = self.create_publisher(PoseStamped, "/goal_pose", 1)
        self.tf = TransformBroadcaster(self)
        self.plans = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(RosPath, "/plan", self.receive_plan, qos)
        coordinates = np.arange(-4.0, 4.01, .1, dtype=np.float32)
        floor = [(x, y, 0.0, 100.0) for x in coordinates for y in coordinates]
        wall = [(3.025, y, z, 100.0) for y in np.arange(-.5, .51, .05)
                for z in np.arange(.025, 1.026, .05)]
        fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
        self.template = point_cloud2.create_cloud(Header(frame_id="camera_init"), fields,
                                                  np.asarray((floor if include_floor else []) + wall, dtype=np.float32))
        self.create_timer(.005, self.publish_odom)
        self.create_timer(.05, self.publish_cloud)

    def receive_plan(self, message):
        if message.poses:
            self.plans.append(message)

    def publish_odom(self):
        odom = Odometry()
        odom.header.frame_id = "camera_init"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

    def publish_cloud(self):
        message = copy.deepcopy(self.template)
        # A small future stamp deliberately requires the next odom bracket.
        message.header.stamp = (self.get_clock().now() + rclpy.duration.Duration(nanoseconds=3000000)).to_msg()
        self.cloud.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--log-dir", required=True)
    parser.add_argument("--floor", action="store_true")
    parser.add_argument("--debug-shutdown", action="store_true")
    args = parser.parse_args()
    directory = Path(args.log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    fixture = StaticFixture(args.floor)
    failures = []
    try:
        for cycle in range(args.cycles):
            fixture.plans.clear()
            logfile = directory / f"navigation_cycle_{cycle + 1}.log"
            with logfile.open("w") as output:
                command = [
                    "ros2", "launch", "navigation", "online_navigation.launch.py",
                    "rviz:=false",
                    "use_sim_time:=false"]
                if args.debug_shutdown:
                    command.append("container_prefix:=" +
                        "gdb -q -batch -ex 'handle SIGINT nostop noprint pass' -ex run -ex bt --args")
                process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                    start_new_session=True)
                success = False
                try:
                    start = time.monotonic()
                    last_goal = 0
                    while time.monotonic() - start < 12 and process.poll() is None:
                        rclpy.spin_once(fixture, timeout_sec=.002)
                        elapsed = time.monotonic() - start
                        if elapsed > 2 and elapsed - last_goal >= 2 and not fixture.plans:
                            goal = PoseStamped()
                            goal.header.frame_id = "camera_init"
                            goal.header.stamp = fixture.get_clock().now().to_msg()
                            goal.pose.position.x = 1.5 + cycle * .05
                            goal.pose.position.y = .5
                            goal.pose.orientation.w = 1.0
                            fixture.goal.publish(goal)
                            last_goal = elapsed
                        if fixture.plans and elapsed > 6:
                            success = True
                            break
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        returncode = process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                        failures.append(f"cycle {cycle + 1}: shutdown timeout")
                        returncode = -9
            text = logfile.read_text()
            crash = any(token in text for token in (
                "exit code -11", "exit code -6", "double free", "corruption", "Segmentation fault", "SIGSEGV"))
            if not success or crash or returncode not in (0, 130):
                failures.append(f"cycle {cycle + 1}: plan={success} crash={crash} returncode={returncode}")
            print(f"CYCLE {cycle + 1} plan_nonempty={success} plan_messages={len(fixture.plans)} "
                  f"shutdown_returncode={returncode} crash={crash}", flush=True)
            for line in text.splitlines():
                if "[Goal]" in line or "[PlanMeta]" in line:
                    print(line, flush=True)
            end = time.monotonic() + .5
            while time.monotonic() < end:
                rclpy.spin_once(fixture, timeout_sec=.005)
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
    print("RESULT", "PASS" if not failures else "FAIL", failures, flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
