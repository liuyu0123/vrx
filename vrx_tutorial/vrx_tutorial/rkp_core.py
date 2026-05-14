#!/usr/bin/env python3
"""RKP core algorithms: A*, Bresenham node reduction, Recursive Kinematic Propagation."""

import heapq
import math
import numpy as np


def a_star_path_planning(grid_map, cost_map, start_rc, goal_rc, search_directions=8, cost_weight=3.0):
    """
    A* path planning on a binary grid (optimized with heapq).

    Args:
        grid_map: 2D numpy array, 0=free, 1=occupied
        cost_map: 2D numpy array, values in [0.0, 1.0] representing inflation
            layer cost. A* adds cost_map[nr,nc] * cost_weight to the step cost
            so the path prefers the middle of channels.
        start_rc: (row, col) tuple, grid index
        goal_rc: (row, col) tuple, grid index
        search_directions: 4 or 8
        cost_weight: multiplier for inflation cost penalty.

    Returns:
        path: Nx2 numpy array of [row, col], or empty array if no path
    """
    rows, cols = grid_map.shape
    start = tuple(start_rc)
    goal = tuple(goal_rc)

    if grid_map[start[0], start[1]] == 1 or grid_map[goal[0], goal[1]] == 1:
        return np.array([])

    # Directions: (dr, dc, cost)
    directions_4 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
    directions_8 = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    ]
    directions = directions_8 if search_directions == 8 else directions_4

    open_heap = []
    heapq.heappush(open_heap, (_heuristic(start, goal), start))
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct_path(came_from, start, goal)

        closed.add(current)
        for dr, dc, step_cost in directions:
            nr, nc = current[0] + dr, current[1] + dc
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue
            if grid_map[nr, nc] == 1:
                continue
            neighbor = (nr, nc)
            tentative_g = g_score[current] + step_cost
            if cost_map is not None:
                tentative_g += float(cost_map[nr, nc]) * cost_weight
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return np.array([])


def _heuristic(a, b):
    """Euclidean heuristic."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct_path(came_from, start, goal):
    """Reconstruct path from came_from dict."""
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return np.array(path, dtype=float)


def bresenham_line(r0, c0, r1, c1):
    """
    Bresenham line algorithm.
    Returns array of [row, col] along the line.
    """
    points = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc

    r, c = r0, c0
    while True:
        points.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc

    return np.array(points, dtype=int)


def bresenham_free(p0, p1, grid_map, cost_map=None, max_cost=0.3):
    """Check if Bresenham line between p0 and p1 is free of obstacles.

    If cost_map is provided, any cell with cost > max_cost is treated as
    blocked so the line stays away from inflated obstacle edges.
    """
    line = bresenham_line(int(round(p0[0])), int(round(p0[1])),
                          int(round(p1[0])), int(round(p1[1])))
    rows, cols = grid_map.shape
    for r, c in line:
        if r < 0 or r >= rows or c < 0 or c >= cols or grid_map[r, c] == 1:
            return False
        if cost_map is not None and cost_map[r, c] > max_cost:
            return False
    return True


def cut_useless_nodes(path, grid_map, cost_map=None, max_look=150, max_cost=0.3):
    """
    Bidirectional node reduction using Bresenham visibility check.
    Mimics Mission.cut_useless_node() behavior.

    Args:
        path: Nx2 array of [row, col]
        grid_map: binary grid
        cost_map: optional normalized cost map for inflation-layer check.
        max_look: max jump distance in nodes
        max_cost: if cost_map is given, any Bresenham cell with cost > max_cost
            blocks the jump, preserving waypoints that keep the path in the
            middle of channels.

    Returns:
        sparse_path: Mx2 array
    """
    if len(path) <= 2:
        return path.copy()

    # Forward pass (similar to cut_useless_node_backward in MATLAB)
    cut = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        find_length = min(max_look, n - 1 - i)
        best_j = i + 1
        for j in range(i + find_length, i, -1):
            if bresenham_free(path[i], path[j], grid_map, cost_map, max_cost):
                best_j = j
                break
        cut.append(path[best_j])
        i = best_j

    cut = np.array(cut)

    # Backward pass (similar to cut_useless_node_forward in MATLAB)
    final = [cut[-1]]
    i = len(cut) - 1
    while i > 0:
        find_length = min(max_look, i)
        best_j = i - 1
        for j in range(i - find_length, i):
            if bresenham_free(cut[i], cut[j], grid_map, cost_map, max_cost):
                best_j = j
                break
        final.append(cut[best_j])
        i = best_j

    final = np.array(final[::-1])
    return final


class RecursiveKinematicPropagator:
    """
    Recursive Kinematic Propagator for USV trajectory smoothing.
    Translated from MATLAB RecursiveKinematicPropagator.m
    """

    def __init__(self, params):
        self.dt = params.get('dt', 0.1)
        self.u = params.get('u', 2.0)
        self.r_max = params.get('r_max', math.radians(25.0))
        self.K_heading = params.get('K_heading', 1.5)
        self.map = params.get('map', None)  # binary grid (0=free, 1=occ)
        self.safe_margin = params.get('safe_margin', 2)
        self.reach_threshold = params.get('reach_threshold', 4.0)
        self.origin_x = params.get('origin_x', 0.0)
        self.origin_y = params.get('origin_y', 0.0)
        self.resolution = params.get('resolution', 1.0)

        # Initial state [x, y, psi, r]
        self.x = np.array(params.get('x0', [0.0, 0.0, 0.0, 0.0]), dtype=float).reshape(4)

    def world_to_grid(self, x, y):
        """Convert world (x,y) to grid (row, col)."""
        c = int((x - self.origin_x) / self.resolution)
        r = int((y - self.origin_y) / self.resolution)
        return r, c

    def is_occupied(self, x, y):
        """Check if world position is occupied."""
        if self.map is None:
            return False
        r, c = self.world_to_grid(x, y)
        rows, cols = self.map.shape
        if r < 0 or r >= rows or c < 0 or c >= cols:
            return False
        return self.map[r, c] == 1

    def run(self, path_waypoints):
        """
        Propagate along waypoints.

        Args:
            path_waypoints: Nx2 array of world coordinates [x, y]

        Returns:
            traj: Mx4 array [x, y, psi, r]
        """
        block_size = 10000
        traj = np.zeros((block_size, 4))
        traj[0, :] = self.x.copy()
        step = 0

        n_targets = len(path_waypoints)
        for i in range(1, n_targets):
            target_raw = path_waypoints[i]
            target = self._sanitize_target(target_raw)

            step_start = step
            while True:
                current_pos = self.x[0:2]
                dist = np.linalg.norm(target - current_pos)

                if i < n_targets - 1:
                    if dist < self.reach_threshold and step > step_start:
                        break
                else:
                    if dist < 3.0:
                        break

                self._propagate_step(target)
                step += 1
                if step >= len(traj):
                    traj = np.vstack([traj, np.zeros((block_size, 4))])
                traj[step, :] = self.x.copy()

                if step - step_start > 100000:
                    break

        return traj[:step + 1, :]

    def _propagate_step(self, target):
        px, py, psi, r = self.x

        psi_target = math.atan2(target[1] - py, target[0] - px)
        e_psi = psi_target - psi
        e_psi = math.atan2(math.sin(e_psi), math.cos(e_psi))

        # Obstacle avoidance heading correction
        if self.map is not None:
            e_psi += self._obstacle_avoidance(px, py, psi)

        r_cmd = self.K_heading * e_psi
        r_cmd = max(min(r_cmd, self.r_max), -self.r_max)

        # Safe yaw rate selection
        if self.map is not None:
            r_cmd = self._select_safe_yaw_rate(px, py, psi, r_cmd)

        px_next = px + self.u * math.cos(psi) * self.dt
        py_next = py + self.u * math.sin(psi) * self.dt
        psi_next = psi + r_cmd * self.dt

        self.x = np.array([px_next, py_next, psi_next, r_cmd])

    def _obstacle_avoidance(self, px, py, psi):
        delta_psi = 0.0
        if self.map is None:
            return delta_psi

        # Front sector check
        check_steps = 5
        sector_angles = [math.pi / 6, 0, -math.pi / 6]
        sector_dists = np.zeros(3)

        for k, sa in enumerate(sector_angles):
            a = psi + sa
            for s in range(1, check_steps + 1):
                d = s * self.u * self.dt
                fx = px + d * math.cos(a)
                fy = py + d * math.sin(a)
                if self.is_occupied(fx, fy):
                    break
                sector_dists[k] = s

        if sector_dists[1] < check_steps:
            if sector_dists[0] > sector_dists[2]:
                delta_psi = math.radians(35)
            elif sector_dists[2] > sector_dists[0]:
                delta_psi = math.radians(-35)
            else:
                delta_psi = math.radians(35)
            return delta_psi

        # Side check
        side_angles = [math.pi / 2, -math.pi / 2]
        side_dists = [float('inf'), float('inf')]
        for k, sa in enumerate(side_angles):
            a = psi + sa
            for s in range(1, 3):
                fx = px + s * math.cos(a)
                fy = py + s * math.sin(a)
                if self.is_occupied(fx, fy):
                    side_dists[k] = s
                    break

        if side_dists[0] < 2 and side_dists[0] < side_dists[1]:
            delta_psi = math.radians(-25)
        elif side_dists[1] < 2 and side_dists[1] < side_dists[0]:
            delta_psi = math.radians(25)

        return delta_psi

    def _select_safe_yaw_rate(self, px, py, psi, r_desired):
        if self.map is None:
            return r_desired

        candidates = [
            r_desired,
            self.r_max, -self.r_max,
            self.r_max * 0.75, -self.r_max * 0.75,
            self.r_max * 0.5, -self.r_max * 0.5,
            self.r_max * 0.25, -self.r_max * 0.25,
            0.0,
        ]
        candidates = list(dict.fromkeys(candidates))  # unique preserving order

        best_r = r_desired
        best_cost = float('inf')

        for r_test in candidates:
            px_sim = px
            py_sim = py
            psi_sim = psi
            collision = False
            for _ in range(3):
                px_sim += self.u * math.cos(psi_sim) * self.dt
                py_sim += self.u * math.sin(psi_sim) * self.dt
                psi_sim += r_test * self.dt

                for dr in range(-self.safe_margin, self.safe_margin + 1):
                    for dc in range(-self.safe_margin, self.safe_margin + 1):
                        if self.is_occupied(px_sim + dr * self.resolution,
                                            py_sim + dc * self.resolution):
                            collision = True
                            break
                    if collision:
                        break
                if collision:
                    break

            if not collision:
                cost = abs(r_test - r_desired)
                if cost < best_cost:
                    best_cost = cost
                    best_r = r_test

        return best_r

    def _sanitize_target(self, target):
        if self.map is None:
            return target

        r0, c0 = self.world_to_grid(target[0], target[1])
        rows, cols = self.map.shape
        search_r = self.safe_margin + 1
        min_dist = float('inf')
        best_dir = np.array([0.0, 0.0])

        for dr in range(-search_r, search_r + 1):
            for dc in range(-search_r, search_r + 1):
                rr = r0 + dr
                cc = c0 + dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    if self.map[rr, cc] == 1:
                        d = math.hypot(dr, dc)
                        if d < min_dist:
                            min_dist = d
                            best_dir = np.array([-dr, -dc], dtype=float)

        if min_dist < float('inf') and min_dist < self.safe_margin + 1:
            norm = np.linalg.norm(best_dir)
            if norm > 0:
                best_dir = best_dir / norm
            push_dist = (self.safe_margin + 1 - min_dist) * self.resolution
            return target + push_dist * best_dir

        return target
