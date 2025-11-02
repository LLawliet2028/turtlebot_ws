#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
from typing import List, Tuple
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf_transformations import euler_from_quaternion  # package: tf_transformations

class DWAConfig:
    def __init__(self):
        # velocities (m/s, rad/s)
        self.max_linear_vel = 0.5
        self.min_linear_vel = 0.0
        self.max_angular_vel = 1.5
        self.min_angular_vel = -1.5
        # accelerations
        self.max_linear_accel = 0.5
        self.max_angular_accel = 1.5
        # sampling
        self.linear_vel_samples = 7
        self.angular_vel_samples = 15
        # prediction
        self.prediction_time = 2.0
        self.dt = 0.1
        # cost weights
        self.goal_cost_weight = 1.0
        self.obstacle_cost_weight = 3.0
        self.velocity_cost_weight = 0.1
        # robot geometry
        self.robot_radius = 0.18
        self.min_obstacle_distance = 0.25
        # goal tolerance
        self.goal_tolerance = 0.15

class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')
        self.config = DWAConfig()

        # state
        self.current_pose = None  # geometry_msgs/Pose
        self.current_twist = None
        self.laser_data: LaserScan = None
        self.goal_pose = None
        self.goal_queue: List = []

        # stagnation tracking across calls
        self.stagnant_counter = 0
        self.last_best_cost = float('inf')

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subscriptions / publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos)
        self.rviz_goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, qos)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.trajectory_vis_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("DWA Local Planner initialized")

    # -------------------- callbacks --------------------
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.goal_queue.append(msg.pose)
        if self.goal_pose is None:
            self.goal_pose = self.goal_queue.pop(0)

    # -------------------- main loop --------------------
    def control_loop(self):
        # require all sensors + a goal
        if not self.is_ready():
            return

        if self.is_goal_reached():
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            if self.goal_queue:
                self.goal_pose = self.goal_queue.pop(0)
                self.get_logger().info(f"Fetching next goal from queue: x={self.goal_pose.position.x:.2f}, y={self.goal_pose.position.y:.2f}")
            else:
                self.goal_pose = None
            # reset stagnation
            self.stagnant_counter = 0
            self.last_best_cost = float('inf')
            return

        best_cmd = self.dwa_planning()
        if best_cmd is not None:
            cmd_msg = Twist()
            cmd_msg.linear.x = float(best_cmd[0])
            cmd_msg.angular.z = float(best_cmd[1])
            self.cmd_vel_pub.publish(cmd_msg)
        else:
            self.stop_robot()
            self.get_logger().warn("No safe trajectory found - stopping robot")

    def is_ready(self) -> bool:
        return all([self.current_pose is not None, self.current_twist is not None, self.goal_pose is not None, self.laser_data is not None])

    def is_goal_reached(self) -> bool:
        if self.goal_pose is None or self.current_pose is None:
            return False
        dx = self.current_pose.position.x - self.goal_pose.position.x
        dy = self.current_pose.position.y - self.goal_pose.position.y
        dist = math.hypot(dx, dy)
        return dist < self.config.goal_tolerance

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    # -------------------- DWA core --------------------
    def dwa_planning(self):
        velocity_window = self.generate_dynamic_window()
        if not velocity_window:
            return None

        best_cost = float('inf')
        best_cmd = None
        all_trajectories = []

        for (lv, av) in velocity_window:
            traj = self.predict_trajectory(lv, av)
            if not traj:
                continue
            cost = self.evaluate_trajectory(traj, lv, av)
            all_trajectories.append((traj, cost, (lv, av)))
            if cost < best_cost:
                best_cost = cost
                best_cmd = (lv, av)

        # stagnation logic (persistent across calls)
        if best_cost == self.last_best_cost:
            self.stagnant_counter += 1
        else:
            self.stagnant_counter = 0
            self.last_best_cost = best_cost

        if self.stagnant_counter >= 25:
            self.get_logger().warning("[DWA] Stagnation detected - no progress for some iterations")
            # return None so robot stops or you may choose a safe fallback
            return None

        # visualize trajectories and highlight the best
        self.visualize_trajectories(all_trajectories, best_cmd)
        return best_cmd

    def generate_dynamic_window(self) -> List[Tuple[float, float]]:
        if self.current_twist is None:
            curr_lv = 0.0
            curr_av = 0.0
        else:
            curr_lv = self.current_twist.linear.x
            curr_av = self.current_twist.angular.z

        dlv = self.config.max_linear_accel * self.config.dt
        dav = self.config.max_angular_accel * self.config.dt

        min_lv = max(self.config.min_linear_vel, curr_lv - dlv)
        max_lv = min(self.config.max_linear_vel, curr_lv + dlv)
        min_av = max(self.config.min_angular_vel, curr_av - dav)
        max_av = min(self.config.max_angular_vel, curr_av + dav)

        # create samples
        lv_samples = self.config.linear_vel_samples
        av_samples = self.config.angular_vel_samples
        if lv_samples <= 1:
            lv_list = [min_lv]
        else:
            lv_list = [min_lv + i * (max_lv - min_lv) / (lv_samples - 1) for i in range(lv_samples)]
        if av_samples <= 1:
            av_list = [min_av]
        else:
            av_list = [min_av + j * (max_av - min_av) / (av_samples - 1) for j in range(av_samples)]

        return [(lv, av) for lv in lv_list for av in av_list]

    def predict_trajectory(self, lv: float, av: float) -> List[Tuple[float, float]]:
        """
        Predict trajectory expressed in world frame (x, y).
        Start from current robot pose (x0, y0, yaw0).
        """
        if self.current_pose is None:
            return []

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # compute yaw from quaternion
        q = self.current_pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        traj = []
        t = 0.0
        steps = max(1, int(self.config.prediction_time / self.config.dt))
        for _ in range(steps):
            # simple unicycle integration
            x += lv * math.cos(yaw) * self.config.dt
            y += lv * math.sin(yaw) * self.config.dt
            yaw += av * self.config.dt
            traj.append((x, y))
        return traj

    # -------------------- cost functions --------------------
    def evaluate_trajectory(self, traj: List[Tuple[float, float]], lv: float, av: float) -> float:
        if not traj:
            return float('inf')
        goal_cost = self.calculate_goal_cost(traj[-1])
        obs_cost = self.calculate_obstacle_cost(traj)
        vel_cost = self.calculate_velocity_cost(lv, av)
        if obs_cost == float('inf'):
            return float('inf')
        total = (self.config.goal_cost_weight * goal_cost +
                 self.config.obstacle_cost_weight * obs_cost +
                 self.config.velocity_cost_weight * vel_cost)
        return total

    def calculate_goal_cost(self, endpoint: Tuple[float, float]) -> float:
        if self.current_pose is None or self.goal_pose is None:
            return float('inf')
        dx = self.goal_pose.position.x - endpoint[0]
        dy = self.goal_pose.position.y - endpoint[1]
        return math.hypot(dx, dy)

    def calculate_obstacle_cost(self, traj: List[Tuple[float, float]]) -> float:
        """
        For each point on the trajectory, find corresponding laser measurement
        and check if collision would happen. Returns inverse distance (higher = worse).
        """
        if self.laser_data is None:
            return 0.0

        min_buffer = float('inf')
        for (x, y) in traj:
            # compute relative vector from robot to this point
            rx = x - self.current_pose.position.x
            ry = y - self.current_pose.position.y
            d = math.hypot(rx, ry)
            angle = math.atan2(ry, rx)

            idx = self.angle_to_laser_index(angle)
            if idx < 0 or idx >= len(self.laser_data.ranges):
                # no data for this angle -> treat as safe (or choose conservative)
                continue

            r = self.laser_data.ranges[idx]
            if not math.isfinite(r):
                continue

            # if laser measured nearer than the predicted point minus robot radius -> collision
            if r + 1e-6 < d + self.config.robot_radius:
                return float('inf')
            # buffer margin
            buffer = r - d - self.config.robot_radius
            if buffer >= 0:
                min_buffer = min(min_buffer, buffer)

        # if closest buffer is large enough, obstacle cost is small (0)
        if min_buffer == float('inf'):
            return 0.0
        if min_buffer < self.config.min_obstacle_distance:
            # higher cost for nearer obstacles
            return 1.0 / max(min_buffer, 1e-3)
        return 0.0

    def angle_to_laser_index(self, angle: float) -> int:
        """
        Convert world angle to index in LaserScan.ranges.
        We must map angle into laser frame: angle relative to robot forward.
        LaserScan.angle_min..angle_max correspond to sensor frame.
        """
        if self.laser_data is None:
            return -1

        # convert angle to sensor frame (range [-pi, pi]) but we must align with laser's start
        # Laser angles are measured from angle_min, incrementing.
        # Normalize angle to [-pi, pi]
        a = angle
        # build relative angle referenced to robot forward, laser angle_min is measured from forward
        # Ensure we wrap angle difference into [0, 2*pi)
        ang = a - self.laser_data.angle_min
        # If we want index: idx = round(ang / angle_increment)
        idx_float = ang / self.laser_data.angle_increment
        idx = int(round(idx_float))
        return max(0, min(idx, len(self.laser_data.ranges) - 1))

    def calculate_velocity_cost(self, lv: float, av: float) -> float:
        # penalize low forward speed slightly, and large angular speed
        return (abs(self.config.max_linear_vel - lv) / max(1e-3, self.config.max_linear_vel) +
                abs(av) / max(1e-3, self.config.max_angular_vel))

    # -------------------- visualization --------------------
    def visualize_trajectories(self, trajectories, best_cmd):
        """
        trajectories: list of (traj, cost, (lv,av))
        """
        marker_array = MarkerArray()
        best_index = -1
        # find index of best_cmd
        if best_cmd is not None:
            for i, (_, _, cmd) in enumerate(trajectories):
                if (abs(cmd[0] - best_cmd[0]) < 1e-6) and (abs(cmd[1] - best_cmd[1]) < 1e-6):
                    best_index = i
                    break

        for i, (traj, cost, cmd) in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "map"  # or "odom" / "base_link" as appropriate
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.color.a = 0.9

            if i == best_index:
                # green for best
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                if cost == float('inf'):
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    norm = min(cost / 10.0, 1.0)
                    marker.color.r = norm
                    marker.color.g = 0.0
                    marker.color.b = 1.0 - norm

            for (px, py) in traj:
                p = Point()
                p.x = float(px)
                p.y = float(py)
                p.z = 0.0
                marker.points.append(p)
            marker_array.markers.append(marker)

        self.trajectory_vis_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    planner = DWALocalPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
