#!/usr/bin/env python3
import csv
import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from swarm_interfaces.msg import PheromoneGrid, TargetDetection


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Ожидаемые цвета (по одному на цвет). Порядок — только для красоты в CSV.
EXPECTED_COLORS = ("red", "green", "blue", "yellow", "magenta")


class ColorCluster:
    """Состояние одной цели одного цвета."""
    __slots__ = ("x", "y", "hits", "t_first", "announced")

    def __init__(self, x: float, y: float, t: float):
        self.x = x
        self.y = y
        self.hits = 1
        self.t_first = t
        self.announced = False


class Metrics(Node):
    def __init__(self):
        super().__init__("swarm_metrics")

        self.declare_parameter("n_drones", 5)
        self.declare_parameter("run_tag", "run01")
        self.declare_parameter("out_dir", "/tmp/swarm_runs")
        self.declare_parameter("coverage_threshold", 0.05)
        self.declare_parameter("expected_targets", 5)
        # Новое:
        self.declare_parameter("min_hits", 2)        # сколько подтверждений нужно
        self.declare_parameter("smooth_alpha", 0.15) # EMA-сглаживание центра

        self.n_drones = int(self.get_parameter("n_drones").value)
        self.run_tag = str(self.get_parameter("run_tag").value)
        self.out_dir = str(self.get_parameter("out_dir").value)
        self.cov_th = float(self.get_parameter("coverage_threshold").value)
        self.expected = int(self.get_parameter("expected_targets").value)
        self.min_hits = int(self.get_parameter("min_hits").value)
        self.alpha = float(self.get_parameter("smooth_alpha").value)

        os.makedirs(self.out_dir, exist_ok=True)
        self.csv_path = os.path.join(self.out_dir, "run_{}.csv".format(self.run_tag))
        self.events_path = os.path.join(self.out_dir, "events_{}.csv".format(self.run_tag))

        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow([
            "t", "n_red", "n_green", "n_blue", "n_yellow", "n_magenta",
            "n_total", "coverage_frac", "path_len_total",
        ])

        self.ev_f = open(self.events_path, "w", newline="")
        self.ev_w = csv.writer(self.ev_f)
        self.ev_w.writerow(["t", "color", "x", "y", "hits"])

        # По одному кластеру на цвет.
        self.clusters: Dict[str, ColorCluster] = {}
        self.announced_order = 0  # счётчик подтверждённых целей

        self.prev_xy: Dict[int, Tuple[float, float]] = {}
        self.path_len: float = 0.0
        self.coverage_frac: float = 0.0
        self.t0: Optional[float] = None
        self.t_first_target: Optional[float] = None
        self.t_all_targets: Optional[float] = None

        self.create_subscription(
            TargetDetection, "/targets/detections", self._on_det, 10)
        self.create_subscription(
            PheromoneGrid, "/pheromones/explored", self._on_cov, 10)
        for i in range(1, self.n_drones + 1):
            self.create_subscription(
                VehicleLocalPosition,
                "/px4_{}/fmu/out/vehicle_local_position".format(i),
                lambda m, did=i: self._on_pos(did, m),
                PX4_QOS,
            )

        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "swarm_metrics: n={}, tag={}, csv={}, min_hits={}, alpha={:.2f}".format(
                self.n_drones, self.run_tag, self.csv_path,
                self.min_hits, self.alpha))

    # ---------- утилиты ----------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ensure_t0(self):
        if self.t0 is None:
            self.t0 = self._now()

    # ---------- колбэки ----------

    def _on_pos(self, did: int, m: VehicleLocalPosition):
        if not m.xy_valid:
            return
        self._ensure_t0()
        xy = (float(m.x), float(m.y))
        if did in self.prev_xy:
            px, py = self.prev_xy[did]
            self.path_len += math.hypot(xy[0] - px, xy[1] - py)
        self.prev_xy[did] = xy

    def _on_det(self, m: TargetDetection):
        self._ensure_t0()
        color = (m.color_label or "").lower()
        if not color:
            return
        tx = float(m.world_position.x)
        ty = float(m.world_position.y)
        t_rel = self._now() - self.t0

        cl = self.clusters.get(color)
        if cl is None:
            # первая детекция этого цвета — создаём кластер, но пока не объявляем
            cl = ColorCluster(tx, ty, t_rel)
            self.clusters[color] = cl
        else:
            # уточняем центр экспоненциальным сглаживанием
            cl.x = (1.0 - self.alpha) * cl.x + self.alpha * tx
            cl.y = (1.0 - self.alpha) * cl.y + self.alpha * ty
            cl.hits += 1

        # объявляем цель уникальной только после min_hits подтверждений
        if (not cl.announced) and cl.hits >= self.min_hits:
            cl.announced = True
            self.announced_order += 1
            n = self.announced_order

            self.ev_w.writerow(["{:.2f}".format(t_rel), color,
                                "{:.2f}".format(cl.x), "{:.2f}".format(cl.y),
                                cl.hits])
            self.ev_f.flush()

            if self.t_first_target is None:
                self.t_first_target = t_rel
                self.get_logger().info(
                    "FIRST target: {} at ({:+.1f},{:+.1f}), t={:.1f}s".format(
                        color, cl.x, cl.y, t_rel))

            self.get_logger().info(
                "unique target #{}: {} at ({:+.1f},{:+.1f}), t={:.1f}s, hits={}".format(
                    n, color, cl.x, cl.y, t_rel, cl.hits))

            if n >= self.expected and self.t_all_targets is None:
                self.t_all_targets = t_rel
                self.get_logger().info(
                    "ALL {} targets found at t={:.1f}s".format(self.expected, t_rel))

    def _on_cov(self, m: PheromoneGrid):
        if not m.data:
            return
        total = len(m.data)
        if total == 0:
            return
        hit = sum(1 for v in m.data if v > self.cov_th)
        self.coverage_frac = hit / float(total)

    # ---------- периодический лог ----------

    def _tick(self):
        if self.t0 is None:
            return
        t_rel = self._now() - self.t0
        by_color = {c: 0 for c in EXPECTED_COLORS}
        n_announced = 0
        for color, cl in self.clusters.items():
            if cl.announced and color in by_color:
                by_color[color] = 1
                n_announced += 1
        self.csv_w.writerow([
            "{:.2f}".format(t_rel),
            by_color["red"], by_color["green"], by_color["blue"],
            by_color["yellow"], by_color["magenta"],
            n_announced,
            "{:.4f}".format(self.coverage_frac),
            "{:.2f}".format(self.path_len),
        ])
        self.csv_f.flush()

    # ---------- финальный отчёт ----------

    def summary(self):
        n_announced = sum(1 for cl in self.clusters.values() if cl.announced)
        self.get_logger().info("=" * 50)
        self.get_logger().info("RUN SUMMARY [{}]".format(self.run_tag))
        self.get_logger().info("  unique targets:  {}/{}".format(
            n_announced, self.expected))
        for color in EXPECTED_COLORS:
            cl = self.clusters.get(color)
            if cl is None:
                self.get_logger().info("    {:<8s} — not seen".format(color))
            elif not cl.announced:
                self.get_logger().info(
                    "    {:<8s} — unstable ({} hits)".format(color, cl.hits))
            else:
                self.get_logger().info(
                    "    {:<8s} at ({:+.1f},{:+.1f}), {} hits".format(
                        color, cl.x, cl.y, cl.hits))
        if self.t_first_target is not None:
            self.get_logger().info("  t_first_target:  {:.1f} s".format(
                self.t_first_target))
        if self.t_all_targets is not None:
            self.get_logger().info("  t_all_targets:   {:.1f} s".format(
                self.t_all_targets))
        self.get_logger().info("  coverage (last): {:.1%}".format(
            self.coverage_frac))
        self.get_logger().info("  path length:     {:.1f} m".format(
            self.path_len))
        self.get_logger().info("  csv:    {}".format(self.csv_path))
        self.get_logger().info("  events: {}".format(self.events_path))
        self.get_logger().info("=" * 50)

    def close(self):
        try:
            self.csv_f.close()
            self.ev_f.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = Metrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
