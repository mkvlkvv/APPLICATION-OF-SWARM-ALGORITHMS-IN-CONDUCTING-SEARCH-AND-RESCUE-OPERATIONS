#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from px4_msgs.msg import VehicleLocalPosition
from swarm_interfaces.msg import PheromoneGrid, TargetDetection
from swarm_interfaces.srv import DepositPheromone

class PheromoneServer(Node):
    def __init__(self):
        super().__init__("pheromone_server")

        self.declare_parameter("origin_x", -25.0)
        self.declare_parameter("origin_y", -25.0)
        self.declare_parameter("cell_size", 1.0)
        self.declare_parameter("width", 50)
        self.declare_parameter("height", 50)
        self.declare_parameter("tau_explored", 60.0)
        self.declare_parameter("tau_target", 300.0)
        self.declare_parameter("deposit_radius_explored", 3.0)
        self.declare_parameter("deposit_radius_target", 2.0)
        self.declare_parameter("explored_deposit_rate", 5.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("viz_rate", 1.0)
        self.declare_parameter("num_agents", 5)
        self.declare_parameter("agent_namespace_prefix", "px4_")
        self.declare_parameter("agent_origins_x", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("agent_origins_y", [0.0, 3.0, -3.0, 6.0, -6.0])

        self.ox = float(self.get_parameter("origin_x").value)
        self.oy = float(self.get_parameter("origin_y").value)
        self.cs = float(self.get_parameter("cell_size").value)
        self.W = int(self.get_parameter("width").value)
        self.H = int(self.get_parameter("height").value)
        self.tau_e = float(self.get_parameter("tau_explored").value)
        self.tau_t = float(self.get_parameter("tau_target").value)
        self.r_e = float(self.get_parameter("deposit_radius_explored").value)
        self.r_t = float(self.get_parameter("deposit_radius_target").value)
        self.num_agents = int(self.get_parameter("num_agents").value)
        self.ns_prefix = str(self.get_parameter("agent_namespace_prefix").value)
        self.ax = list(self.get_parameter("agent_origins_x").value)
        self.ay = list(self.get_parameter("agent_origins_y").value)

        self.explored = np.zeros((self.H, self.W), dtype=np.float32)
        self.target = np.zeros((self.H, self.W), dtype=np.float32)
        self.last_evap = self.get_clock().now()
        self.agent_pos = {i: None for i in range(1, self.num_agents + 1)}

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for i in range(1, self.num_agents + 1):
            topic = f"/{self.ns_prefix}{i}/fmu/out/vehicle_local_position"
            self.create_subscription(
                VehicleLocalPosition, topic,
                lambda msg, aid=i: self.on_agent_pos(aid, msg),
                qos_px4,
            )

        self.create_subscription(
            TargetDetection, "/targets/detections",
            self.on_detection, qos_rel,
        )

        self.pub_explored = self.create_publisher(
            PheromoneGrid, "/pheromones/explored", qos_rel)
        self.pub_target = self.create_publisher(
            PheromoneGrid, "/pheromones/target", qos_rel)
        self.pub_explored_viz = self.create_publisher(
            OccupancyGrid, "/pheromones/explored_viz", qos_rel)
        self.pub_target_viz = self.create_publisher(
            OccupancyGrid, "/pheromones/target_viz", qos_rel)

        self.srv_dep_explored = self.create_service(
            DepositPheromone, "/pheromone/deposit_explored",
            self.srv_deposit_explored)
        self.srv_dep_target = self.create_service(
            DepositPheromone, "/pheromone/deposit_target",
            self.srv_deposit_target)

        rate_dep = float(self.get_parameter("explored_deposit_rate").value)
        rate_pub = float(self.get_parameter("publish_rate").value)
        rate_viz = float(self.get_parameter("viz_rate").value)
        self.create_timer(1.0 / rate_dep, self.tick_auto_deposit)
        self.create_timer(1.0 / rate_pub, self.publish_grids)
        self.create_timer(1.0 / rate_viz, self.publish_viz)

        self.get_logger().info(
            f"pheromone_server: {self.W}x{self.H} cells, "
            f"cell={self.cs} m, origin=({self.ox},{self.oy}), "
            f"agents={self.num_agents}"
        )

    def world_to_cell(self, x, y):
        i = int((y - self.oy) / self.cs)
        j = int((x - self.ox) / self.cs)
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.H and 0 <= j < self.W

    def evaporate(self):
        now = self.get_clock().now()
        dt = (now - self.last_evap).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_evap = now
        self.explored *= math.exp(-dt / self.tau_e)
        self.target *= math.exp(-dt / self.tau_t)

    def deposit(self, grid, x, y, amount, radius):
        if not (self.ox <= x < self.ox + self.W * self.cs):
            return False
        if not (self.oy <= y < self.oy + self.H * self.cs):
            return False
        ci, cj = self.world_to_cell(x, y)
        sigma = max(radius / 2.0, self.cs)
        rad_cells = int(math.ceil(2 * sigma / self.cs))
        for di in range(-rad_cells, rad_cells + 1):
            for dj in range(-rad_cells, rad_cells + 1):
                i, j = ci + di, cj + dj
                if not self.in_bounds(i, j):
                    continue
                dx = di * self.cs
                dy = dj * self.cs
                w = math.exp(-(dx * dx + dy * dy) / (2 * sigma * sigma))
                grid[i, j] = min(1.0, grid[i, j] + amount * w)
        return True

    def on_agent_pos(self, aid, msg):
        if not msg.xy_valid:
            return
        ox = self.ax[aid - 1] if aid - 1 < len(self.ax) else 0.0
        oy = self.ay[aid - 1] if aid - 1 < len(self.ay) else 0.0
        wx = msg.x + ox
        wy = msg.y + oy
        self.agent_pos[aid] = (wx, wy, msg.z)

    def on_detection(self, msg):
        x = msg.world_position.x
        y = msg.world_position.y
        amt = float(max(0.1, min(1.0, msg.confidence)))
        self.evaporate()
        self.deposit(self.target, x, y, amt, self.r_t)

    def tick_auto_deposit(self):
        self.evaporate()
        for aid, p in self.agent_pos.items():
            if p is None:
                continue
            wx, wy, _ = p
            self.deposit(self.explored, wx, wy, 0.3, self.r_e)

    def srv_deposit_explored(self, req, resp):
        self.evaporate()
        resp.success = self.deposit(
            self.explored, req.x, req.y, req.amount, self.r_e)
        return resp

    def srv_deposit_target(self, req, resp):
        self.evaporate()
        resp.success = self.deposit(
            self.target, req.x, req.y, req.amount, self.r_t)
        return resp

    def _make_grid_msg(self, grid):
        m = PheromoneGrid()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.origin_x = float(self.ox)
        m.origin_y = float(self.oy)
        m.cell_size = float(self.cs)
        m.width = int(self.W)
        m.height = int(self.H)
        m.data = grid.flatten().astype(np.float32).tolist()
        return m

    def _make_occ_msg(self, grid):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        md = MapMetaData()
        md.resolution = float(self.cs)
        md.width = int(self.W)
        md.height = int(self.H)
        md.origin = Pose()
        md.origin.position.x = float(self.ox)
        md.origin.position.y = float(self.oy)
        md.origin.position.z = 0.0
        md.origin.orientation.w = 1.0
        m.info = md
        occ = np.clip(grid * 100.0, 0, 100).astype(np.int8).flatten()
        m.data = occ.tolist()
        return m

    def publish_grids(self):
        self.evaporate()
        self.pub_explored.publish(self._make_grid_msg(self.explored))
        self.pub_target.publish(self._make_grid_msg(self.target))

    def publish_viz(self):
        self.pub_explored_viz.publish(self._make_occ_msg(self.explored))
        self.pub_target_viz.publish(self._make_occ_msg(self.target))

def main():
    rclpy.init()
    node = PheromoneServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()