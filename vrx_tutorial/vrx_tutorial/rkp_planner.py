#!/usr/bin/env python3
"""RKP Global Planner Node for VRX Tutorial.

Provides a ComputePathToPose action server that replaces Nav2's default
planner with the Recursive Kinematic Propagation (RKP) pipeline:
    A* -> Bresenham node reduction -> RKP smoothing.

Integrate with Nav2 by setting the behavior-tree ComputePathToPose node to:
    server_name="/rkp/ComputePathToPose"
"""

import asyncio
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
import tf2_ros
from builtin_interfaces.msg import Duration

from vrx_tutorial.rkp_core import (
    a_star_path_planning,
    cut_useless_nodes,
    RecursiveKinematicPropagator,
)


class RKPPlanner(Node):
    def __init__(self):
        super().__init__('rkp_planner', namespace='rkp')

        # Parameters
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('robot_base_frame', 'wamv/wamv/base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('planner_timeout_sec', 10.0)

        # RKP parameters
        self.declare_parameter('rkp.dt', 0.1)
        self.declare_parameter('rkp.u', 2.0)
        self.declare_parameter('rkp.r_max_deg', 25.0)
        self.declare_parameter('rkp.K_heading', 1.5)
        self.declare_parameter('rkp.safe_margin', 2)
        self.declare_parameter('rkp.reach_threshold', 4.0)
        self.declare_parameter('astar.search_directions', 8)
        self.declare_parameter('node_reduction.max_look', 150)
        self.declare_parameter('node_reduction.enabled', True)
        self.declare_parameter('node_reduction.min_waypoints', 4)

        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.planner_timeout_sec = self.get_parameter('planner_timeout_sec').value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Costmap subscription
        self._latest_costmap = None
        self._costmap_lock = False
        self._planning_lock = asyncio.Lock()
        costmap_topic = self.get_parameter('global_costmap_topic').value
        self.create_subscription(OccupancyGrid, costmap_topic, self._costmap_callback, 1)

        # Path publisher for RViz (publish to /plan so default Nav2 display sees it)
        self._path_pub = self.create_publisher(Path, '/plan', 10)
        self._path_debug_pub = self.create_publisher(Path, '/rkp/plan', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self._execute_callback,
        )

        self.get_logger().info('RKP planner started. Waiting for costmap...')

    # ------------------------------------------------------------------
    # Costmap handling
    # ------------------------------------------------------------------
    def _costmap_callback(self, msg: OccupancyGrid):
        # Convert OccupancyGrid to binary numpy grid
        # Values: 0-100 = probability, -1 = unknown
        # In vrx nav2_params, track_unknown_space=false, so unknown is treated as free.
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        grid = np.where(data > 50, 1, 0).astype(np.uint8)

        self._latest_costmap = {
            'grid': grid,
            'info': msg.info,
            'stamp': msg.header.stamp,
        }

    # ------------------------------------------------------------------
    # Coordinate utilities
    # ------------------------------------------------------------------
    def _pose_to_world(self, pose_stamped: PoseStamped):
        """Return (x, y, yaw) in world coordinates."""
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        return p.x, p.y, yaw

    def _world_to_grid(self, x, y, info):
        """World (x,y) -> grid (row, col)."""
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        return r, c

    def _grid_to_world(self, r, c, info):
        """Grid (row, col) -> world (x, y) at cell center."""
        x = info.origin.position.x + (c + 0.5) * info.resolution
        y = info.origin.position.y + (r + 0.5) * info.resolution
        return x, y

    # ------------------------------------------------------------------
    # Action execute
    # ------------------------------------------------------------------
    async def _execute_callback(self, goal_handle):
        async with self._planning_lock:
            start_time = time.time()
            request = goal_handle.request

            # Use start pose from request or look up current robot pose
            if request.use_start and request.start.header.frame_id:
                start_pose = request.start
            else:
                start_pose = self._get_current_pose()
                if start_pose is None:
                    self.get_logger().error('Failed to get current robot pose from TF')
                    goal_handle.abort()
                    result = ComputePathToPose.Result()
                    return result

            goal_pose = request.goal

            # Wait for costmap
            costmap = await self._wait_for_costmap(timeout_sec=5.0)
            if costmap is None:
                self.get_logger().error('No costmap available')
                goal_handle.abort()
                result = ComputePathToPose.Result()
                return result

            grid = costmap['grid']
            info = costmap['info']

            # Convert poses to grid coordinates
            sx, sy, _ = self._pose_to_world(start_pose)
            gx, gy, goal_yaw = self._pose_to_world(goal_pose)

            s_r, s_c = self._world_to_grid(sx, sy, info)
            g_r, g_c = self._world_to_grid(gx, gy, info)

            rows, cols = grid.shape
            # Clamp to map bounds
            s_r = max(0, min(rows - 1, s_r))
            s_c = max(0, min(cols - 1, s_c))
            g_r = max(0, min(rows - 1, g_r))
            g_c = max(0, min(cols - 1, g_c))

            self.get_logger().info(
                f'Planning from grid ({s_r},{s_c}) to ({g_r},{g_c}) | '
                f'world ({sx:.1f},{sy:.1f}) -> ({gx:.1f},{gy:.1f})'
            )

            # 1. A* search
            search_dirs = self.get_parameter('astar.search_directions').value
            path_grid = a_star_path_planning(grid, (s_r, s_c), (g_r, g_c), search_dirs)
            if path_grid.size == 0:
                self.get_logger().error('A* failed to find a path')
                goal_handle.abort()
                result = ComputePathToPose.Result()
                return result

            self.get_logger().info(f'A* found {len(path_grid)} waypoints')

            # 2. Node reduction (Bresenham) or fixed subsampling
            use_reduction = self.get_parameter('node_reduction.enabled').value
            min_waypoints = self.get_parameter('node_reduction.min_waypoints').value
            if use_reduction:
                max_look = self.get_parameter('node_reduction.max_look').value
                sparse_grid = cut_useless_nodes(path_grid, grid, max_look)
                self.get_logger().info(f'Node reduction: {len(path_grid)} -> {len(sparse_grid)}')
                # Fallback if too aggressively reduced (likely costmap missing obstacles)
                if len(sparse_grid) < min_waypoints:
                    self.get_logger().warn(
                        f'Node reduction produced only {len(sparse_grid)} waypoints '
                        f'(< {min_waypoints}). Falling back to subsampled A* path.'
                    )
                    step = max(1, len(path_grid) // min_waypoints)
                    sparse_grid = path_grid[::step].copy()
                    if not np.array_equal(sparse_grid[-1], path_grid[-1]):
                        sparse_grid = np.vstack([sparse_grid, path_grid[-1]])
            else:
                step = max(1, len(path_grid) // min_waypoints)
                sparse_grid = path_grid[::step].copy()
                if not np.array_equal(sparse_grid[-1], path_grid[-1]):
                    sparse_grid = np.vstack([sparse_grid, path_grid[-1]])
                self.get_logger().info(f'Node reduction disabled, subsampled: {len(path_grid)} -> {len(sparse_grid)}')

            # Convert sparse waypoints to world coordinates
            waypoints_world = np.array([
                self._grid_to_world(r, c, info) for r, c in sparse_grid
            ])

            # 3. RKP smoothing
            # Initial heading points from start to second waypoint
            if len(waypoints_world) >= 2:
                dx = waypoints_world[1, 0] - waypoints_world[0, 0]
                dy = waypoints_world[1, 1] - waypoints_world[0, 1]
                psi0 = math.atan2(dy, dx)
            else:
                psi0 = 0.0

            # Start state aligned with current robot yaw if available
            _, _, current_yaw = self._pose_to_world(start_pose)
            # Blend current yaw with waypoint direction (optional; here use waypoint dir)
            psi0 = current_yaw

            rkp_params = {
                'dt': self.get_parameter('rkp.dt').value,
                'u': self.get_parameter('rkp.u').value,
                'r_max': math.radians(self.get_parameter('rkp.r_max_deg').value),
                'K_heading': self.get_parameter('rkp.K_heading').value,
                'safe_margin': self.get_parameter('rkp.safe_margin').value,
                'reach_threshold': self.get_parameter('rkp.reach_threshold').value,
                'x0': [sx, sy, psi0, 0.0],
                'map': grid,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'resolution': info.resolution,
            }

            rkp = RecursiveKinematicPropagator(rkp_params)
            traj = rkp.run(waypoints_world)

            self.get_logger().info(f'RKP generated {len(traj)} trajectory points')

            # 4. Build nav_msgs/Path
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.global_frame

            for pt in traj:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(pt[0])
                pose.pose.position.y = float(pt[1])
                pose.pose.position.z = 0.0
                # Orientation from yaw
                yaw = float(pt[2])
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path_msg.poses.append(pose)

            # Publish for RViz
            self._path_pub.publish(path_msg)
            self._path_debug_pub.publish(path_msg)

            if path_msg.poses:
                first = path_msg.poses[0].pose.position
                last = path_msg.poses[-1].pose.position
                self.get_logger().info(
                    f'Published path: first=({first.x:.2f},{first.y:.2f}) '
                    f'last=({last.x:.2f},{last.y:.2f}) poses={len(path_msg.poses)}'
                )

            # Success
            goal_handle.succeed()
            result = ComputePathToPose.Result()
            result.path = path_msg
            elapsed = time.time() - start_time
            result.planning_time = Duration(sec=int(elapsed), nanosec=int((elapsed % 1) * 1e9))
            self.get_logger().info(f'Planning succeeded in {elapsed:.3f}s')
            return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _get_current_pose(self) -> PoseStamped:
        """Lookup robot pose in global frame via TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0),
            )
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    async def _wait_for_costmap(self, timeout_sec=5.0):
        start = time.time()
        while self._latest_costmap is None and (time.time() - start) < timeout_sec:
            await asyncio.sleep(0.1)
        return self._latest_costmap


def main(args=None):
    rclpy.init(args=args)
    node = RKPPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
