#!/usr/bin/env python3
import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, PointStamped
from px4_msgs.msg import VehicleLocalPosition
from swarm_interfaces.msg import PheromoneGrid, TargetDetection

STATE_SEARCH = 0
STATE_RECRUIT = 1
STATE_INSPECT = 2
STATE_NAMES = {0: "SEARCH", 1: "RECRUIT", 2: "INSPECT"}


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")

        self.declare_parameter("drone_id", 1)
        self.declare_parameter("altitude", 8.0)
        self.declare_parameter("arena_xmin", -20.0)
        self.declare_parameter("arena_xmax", 20.0)
        self.declare_parameter("arena_ymin", -20.0)
        self.declare_parameter("arena_ymax", 20.0)
        self.declare_parameter("levy_xmin", 3.0)
        self.declare_parameter("levy_xmax", 12.0)
        self.declare_parameter("levy_alpha", 1.5)
        self.declare_parameter("gradient_weight", 0.6)
        self.declare_parameter("wp_reach_tol", 1.5)
        self.declare_parameter("recruit_radius", 15.0)
        # теперь это МИНИМАЛЬНЫЙ КОНТРАСТ (peak - background) в окне
        self.declare_parameter("recruit_threshold", 0.15)
        self.declare_parameter("self_target_radius", 6.0)
        self.declare_parameter("known_target_radius", 3.0)
        self.declare_parameter("inspect_duration", 4.0)
        self.declare_parameter("tick_period", 1.0)
        self.declare_parameter("peer_margin", 2.0)
        self.declare_parameter("peer_timeout", 3.0)
        self.declare_parameter("recruit_ring_min", 4.0)
        self.declare_parameter("recruit_min_cells", 3)
        self.declare_parameter("recruit_empty_ticks", 3)
        # доля контраста, выше которой клетки считаются «пиком»
        self.declare_parameter("recruit_peak_frac", 0.5)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.altitude = float(self.get_parameter("altitude").value)
        self.xmin = float(self.get_parameter("arena_xmin").value)
        self.xmax = float(self.get_parameter("arena_xmax").value)
        self.ymin = float(self.get_parameter("arena_ymin").value)
        self.ymax = float(self.get_parameter("arena_ymax").value)
        self.levy_xmin = float(self.get_parameter("levy_xmin").value)
        self.levy_xmax = float(self.get_parameter("levy_xmax").value)
        self.levy_alpha = float(self.get_parameter("levy_alpha").value)
        self.grad_w = float(self.get_parameter("gradient_weight").value)
        self.wp_tol = float(self.get_parameter("wp_reach_tol").value)
        self.recruit_r = float(self.get_parameter("recruit_radius").value)
        self.recruit_thr = float(self.get_parameter("recruit_threshold").value)
        self.self_r = float(self.get_parameter("self_target_radius").value)
        self.known_r = float(self.get_parameter("known_target_radius").value)
        self.inspect_dur = float(self.get_parameter("inspect_duration").value)
        self.tick_dt = float(self.get_parameter("tick_period").value)
        self.peer_margin = float(self.get_parameter("peer_margin").value)
        self.peer_timeout = float(self.get_parameter("peer_timeout").value)
        self.ring_min = float(self.get_parameter("recruit_ring_min").value)
        self.min_cells = int(self.get_parameter("recruit_min_cells").value)
        self.empty_ticks_max = int(self.get_parameter("recruit_empty_ticks").value)
        self.peak_frac = float(self.get_parameter("recruit_peak_frac").value)

        ns = "/px4_{}".format(self.drone_id)

        qos_px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, depth=5)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_valid = False
        self.waypoint = None
        self.state = STATE_SEARCH
        self.inspect_t0 = None
        self.grid_explored = None
        self.grid_target = None
        self.known_targets = []
        self.peers = {}
        self._last_diag = 0.0
        self._empty_recruit_ticks = 0

        self.create_subscription(VehicleLocalPosition,
                                 ns + "/fmu/out/vehicle_local_position",
                                 self._on_pos, qos_px4)
        self.create_subscription(PheromoneGrid, "/pheromones/explored",
                                 self._on_explored, qos_rel)
        self.create_subscription(PheromoneGrid, "/pheromones/target",
                                 self._on_target, qos_rel)
        self.create_subscription(TargetDetection, "/targets/detections",
                                 self._on_detection, qos_rel)
        self.create_subscription(PointStamped, "/swarm/agent_positions",
                                 self._on_peer, 10)

        self.pub_goto = self.create_publisher(Point, ns + "/goto", 10)
        self.pos_pub = self.create_publisher(PointStamped,
                                             "/swarm/agent_positions", 10)

        self.create_timer(self.tick_dt, self._tick)
        self.create_timer(0.5, self._publish_my_pos)

        self.get_logger().info(
            "explorer drone_id={} alt={}m arena=[{},{}]x[{},{}]".format(
                self.drone_id, self.altitude,
                self.xmin, self.xmax, self.ymin, self.ymax))

    # ---------- callbacks ----------

    def _on_pos(self, m):
        if m.xy_valid:
            self.pos_x = float(m.x)
            self.pos_y = float(m.y)
            self.pos_valid = True

    def _on_explored(self, m):
        self.grid_explored = m

    def _on_target(self, m):
        self.grid_target = m

    def _on_peer(self, msg):
        fid = msg.header.frame_id
        if not fid.startswith("drone_"):
            return
        try:
            did = int(fid.split("_")[1])
        except (ValueError, IndexError):
            return
        if did == self.drone_id:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self.peers[did] = (float(msg.point.x), float(msg.point.y), t)

    def _publish_my_pos(self):
        if not self.pos_valid:
            return
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "drone_{}".format(self.drone_id)
        m.point.x = float(self.pos_x)
        m.point.y = float(self.pos_y)
        m.point.z = 0.0
        self.pos_pub.publish(m)

    def _on_detection(self, m):
        if m.agent_id != self.drone_id:
            return
        tx = float(m.world_position.x)
        ty = float(m.world_position.y)
        d = math.hypot(tx - self.pos_x, ty - self.pos_y)
        self.get_logger().info(
            "detection {} at ({:+.1f},{:+.1f}) d={:.1f} self_r={:.1f} state={}".format(
                m.color_label, tx, ty, d, self.self_r, STATE_NAMES[self.state]))
        if d >= self.self_r:
            self.get_logger().info("  rejected: too far")
            return
        if self.state == STATE_INSPECT:
            self.get_logger().info("  rejected: already inspecting")
            return
        if self._is_known(tx, ty):
            self.get_logger().info("  rejected: already known")
            # если физически стоим на известной цели и не инспектируем —
            # не застреваем: принудительно выбираем новый поисковый wp
            if self.state != STATE_INSPECT and d < max(self.wp_tol, 1.5):
                self.waypoint = self._new_search_waypoint()
                self.state = STATE_SEARCH
                self._empty_recruit_ticks = 0
                self.get_logger().info(
                    "  stuck on known target, forced new wp ({:+.1f},{:+.1f})".format(
                        self.waypoint[0], self.waypoint[1]))
                self._publish_waypoint()
            return

        self.known_targets.append((tx, ty))
        self.get_logger().info(
            "registered known target #{} at ({:+.1f},{:+.1f}) -> INSPECT".format(
                len(self.known_targets), tx, ty))

        self.state = STATE_INSPECT
        self.inspect_t0 = self.get_clock().now().nanoseconds * 1e-9
        self.waypoint = (tx, ty)
        self._empty_recruit_ticks = 0
        self._publish_waypoint()

    # ---------- helpers ----------

    @staticmethod
    def _world_to_cell(grid, x, y):
        col = int((x - grid.origin_x) / grid.cell_size)
        row = int((y - grid.origin_y) / grid.cell_size)
        if 0 <= col < grid.width and 0 <= row < grid.height:
            return row, col
        return None

    @staticmethod
    def _cell_to_world(grid, row, col):
        wx = grid.origin_x + (col + 0.5) * grid.cell_size
        wy = grid.origin_y + (row + 0.5) * grid.cell_size
        return wx, wy

    def _grid_as_array(self, grid):
        return np.asarray(grid.data, dtype=np.float32).reshape(grid.height, grid.width)

    def _is_known(self, x, y):
        return any(math.hypot(x - kx, y - ky) < self.known_r
                   for kx, ky in self.known_targets)

    def _peer_closer(self, tx, ty, my_d):
        # если мы уже ведём рекрут — сопротивляемся передаче цели
        margin = self.peer_margin
        if self.state == STATE_RECRUIT:
            margin *= 2.0
        t_now = self.get_clock().now().nanoseconds * 1e-9
        for did, (px, py, t_last) in self.peers.items():
            if t_now - t_last > self.peer_timeout:
                continue
            their_d = math.hypot(tx - px, ty - py)
            if their_d + margin < my_d:
                return True
        return False

    def _find_recruit_target(self):
        if self.grid_target is None:
            return None
        g = self.grid_target
        arr = self._grid_as_array(g)

        cxy = self._world_to_cell(g, self.pos_x, self.pos_y)
        if cxy is None:
            return None
        cr, cc = cxy

        r_cells = int(math.ceil(self.recruit_r / g.cell_size))
        r0 = max(0, cr - r_cells)
        r1 = min(g.height, cr + r_cells + 1)
        c0 = max(0, cc - r_cells)
        c1 = min(g.width, cc + r_cells + 1)

        sub = arr[r0:r1, c0:c1]
        if sub.size == 0:
            return None

        # контраст относительно фона окна вместо абсолютного порога:
        # это работает и на плоском «холодном», и на насыщенном «горячем» поле
        bg = float(np.median(sub))
        peak = float(sub.max())
        contrast = peak - bg
        if contrast < self.recruit_thr:
            return None

        dyn_thr = bg + self.peak_frac * contrast
        mask = sub >= dyn_thr
        if int(mask.sum()) < self.min_cells:
            return None

        rr, cc2 = np.nonzero(mask)
        wxs = g.origin_x + (cc2 + c0 + 0.5) * g.cell_size
        wys = g.origin_y + (rr + r0 + 0.5) * g.cell_size
        vals = sub[rr, cc2].astype(np.float64)

        dxs = wxs - self.pos_x
        dys = wys - self.pos_y
        ds = np.hypot(dxs, dys)

        ring = (ds >= self.ring_min) & (ds <= self.recruit_r)
        if not ring.any():
            return None
        wxs, wys, vals, ds = wxs[ring], wys[ring], vals[ring], ds[ring]

        if self.known_targets:
            keep = np.ones(len(wxs), dtype=bool)
            for kx, ky in self.known_targets:
                keep &= np.hypot(wxs - kx, wys - ky) >= self.known_r
            if not keep.any():
                return None
            wxs, wys, vals, ds = wxs[keep], wys[keep], vals[keep], ds[keep]

        # скор — контраст к фону, нормированный на расстояние
        score = (vals - bg) / (1.0 + ds)
        i = int(np.argmax(score))
        wx, wy, d = float(wxs[i]), float(wys[i]), float(ds[i])

        if d < self.self_r:
            return None
        if self._peer_closer(wx, wy, d):
            return None
        return (wx, wy)

    def _levy_step_length(self):
        u = random.random()
        a = self.levy_alpha
        xmin_a = self.levy_xmin ** (-a)
        xmax_a = self.levy_xmax ** (-a)
        val = (xmin_a - u * (xmin_a - xmax_a)) ** (-1.0 / a)
        return float(np.clip(val, self.levy_xmin, self.levy_xmax))

    def _explored_gradient(self):
        if self.grid_explored is None:
            return 0.0, 0.0
        g = self.grid_explored
        arr = self._grid_as_array(g)
        cxy = self._world_to_cell(g, self.pos_x, self.pos_y)
        if cxy is None:
            return 0.0, 0.0
        cr, cc = cxy
        left  = arr[cr, cc - 1] if cc > 0 else arr[cr, cc]
        right = arr[cr, cc + 1] if cc < g.width - 1 else arr[cr, cc]
        down  = arr[cr - 1, cc] if cr > 0 else arr[cr, cc]
        up    = arr[cr + 1, cc] if cr < g.height - 1 else arr[cr, cc]
        gx = right - left
        gy = up - down
        # идём в сторону НЕисследованного (меньших значений)
        ax, ay = -gx, -gy
        n = math.hypot(ax, ay)
        if n < 1e-6:
            return 0.0, 0.0
        return ax / n, ay / n

    def _clip_to_arena(self, x, y):
        return (float(np.clip(x, self.xmin, self.xmax)),
                float(np.clip(y, self.ymin, self.ymax)))

    def _border_repulsion(self):
        margin = 3.0
        rx, ry = 0.0, 0.0
        dl = self.pos_x - self.xmin
        dr = self.xmax - self.pos_x
        db = self.pos_y - self.ymin
        dt = self.ymax - self.pos_y
        if dl < margin:
            rx += (margin - dl) / margin
        if dr < margin:
            rx -= (margin - dr) / margin
        if db < margin:
            ry += (margin - db) / margin
        if dt < margin:
            ry -= (margin - dt) / margin
        n = math.hypot(rx, ry)
        if n < 1e-6:
            return 0.0, 0.0
        return rx / n, ry / n

    def _new_search_waypoint(self):
        L = self._levy_step_length()
        gx, gy = self._explored_gradient()
        bx, by = self._border_repulsion()
        ang = random.uniform(-math.pi, math.pi)
        rx, ry = math.cos(ang), math.sin(ang)

        if math.hypot(bx, by) > 0.0:
            dx = 0.7 * bx + 0.3 * rx
            dy = 0.7 * by + 0.3 * ry
        else:
            dx = self.grad_w * gx + (1.0 - self.grad_w) * rx
            dy = self.grad_w * gy + (1.0 - self.grad_w) * ry

        n = math.hypot(dx, dy)
        if n < 1e-6:
            dx, dy = rx, ry
            n = 1.0
        dx, dy = dx / n, dy / n
        wx = self.pos_x + L * dx
        wy = self.pos_y + L * dy
        return self._clip_to_arena(wx, wy)

    def _publish_waypoint(self):
        if self.waypoint is None:
            return
        p = Point()
        p.x = float(self.waypoint[0])
        p.y = float(self.waypoint[1])
        p.z = float(self.altitude)
        self.pub_goto.publish(p)

    def _reached(self):
        if self.waypoint is None:
            return True
        dx = self.waypoint[0] - self.pos_x
        dy = self.waypoint[1] - self.pos_y
        return math.hypot(dx, dy) < self.wp_tol

    def _diag_recruit(self, now):
        if self.grid_target is None:
            return
        if now - self._last_diag <= 5.0:
            return
        g = self.grid_target
        arr = self._grid_as_array(g)
        cxy = self._world_to_cell(g, self.pos_x, self.pos_y)
        if cxy is None:
            return
        cr, cc = cxy
        r_cells = int(math.ceil(self.recruit_r / g.cell_size))
        r0 = max(0, cr - r_cells)
        r1 = min(g.height, cr + r_cells + 1)
        c0 = max(0, cc - r_cells)
        c1 = min(g.width, cc + r_cells + 1)
        sub = arr[r0:r1, c0:c1]
        if sub.size == 0:
            return
        bg = float(np.median(sub))
        peak = float(sub.max())
        self.get_logger().info(
            "no recruit: bg={:.3f} peak={:.3f} contrast={:.3f} thr={:.2f}".format(
                bg, peak, peak - bg, self.recruit_thr))
        self._last_diag = now

    def _tick(self):
        if not self.pos_valid:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.state == STATE_INSPECT:
            if self.inspect_t0 is not None and now - self.inspect_t0 > self.inspect_dur:
                self.get_logger().info("INSPECT done -> SEARCH")
                self.state = STATE_SEARCH
                self.inspect_t0 = None
                self.waypoint = None
                self._empty_recruit_ticks = 0
            else:
                self._publish_waypoint()
            return

        recruit_xy = self._find_recruit_target()

        if recruit_xy is None:
            self._diag_recruit(now)

        if recruit_xy is not None:
            self._empty_recruit_ticks = 0
            d = math.hypot(recruit_xy[0] - self.pos_x,
                           recruit_xy[1] - self.pos_y)
            if d < 2.0:
                # пришли в пик феромона, но детектора нет — помечаем «фантом»,
                # чтобы не кружить на месте
                self.known_targets.append(recruit_xy)
                self.get_logger().info(
                    "recruit reached, marked known #{} at ({:+.1f},{:+.1f}) -> SEARCH".format(
                        len(self.known_targets),
                        recruit_xy[0], recruit_xy[1]))
                self.state = STATE_SEARCH
                self.waypoint = None
            else:
                changed = (self.waypoint is None or
                           math.hypot(self.waypoint[0] - recruit_xy[0],
                                      self.waypoint[1] - recruit_xy[1]) > 0.5)
                if self.state != STATE_RECRUIT or changed:
                    self.get_logger().info(
                        "RECRUIT to ({:+.1f},{:+.1f})".format(
                            recruit_xy[0], recruit_xy[1]))
                self.state = STATE_RECRUIT
                self.waypoint = recruit_xy
                self._publish_waypoint()
            return

        # цели нет; если были в RECRUIT — держим waypoint ещё немного
        if self.state == STATE_RECRUIT:
            self._empty_recruit_ticks += 1
            if self._empty_recruit_ticks < self.empty_ticks_max:
                self._publish_waypoint()
                return

        if self.state != STATE_SEARCH:
            self.get_logger().info("-> SEARCH")
            self.state = STATE_SEARCH
            self.waypoint = None
        self._empty_recruit_ticks = 0

        if self.waypoint is None or self._reached():
            self.waypoint = self._new_search_waypoint()
            self.get_logger().info(
                "SEARCH new wp ({:+.1f},{:+.1f}) from ({:+.1f},{:+.1f})".format(
                    self.waypoint[0], self.waypoint[1],
                    self.pos_x, self.pos_y))
        self._publish_waypoint()


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()