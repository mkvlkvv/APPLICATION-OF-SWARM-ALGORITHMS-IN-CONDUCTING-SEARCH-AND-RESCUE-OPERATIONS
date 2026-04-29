#!/usr/bin/env python3
"""
target_detector.py — детектор цветных целей по нижней камере БПЛА.

Алгоритм:
  1. BGR → HSV.
  2. Для каждого эталонного цвета — inRange по HSV-диапазону.
  3. Морфология (open+close) для удаления шумов.
  4. findContours → отбор по площади [min_area .. max_area].
  5. Центр масс контура (u, v) → луч в системе камеры → поворот в body → 
     поворот в world (NED) полным кватернионом → пересечение с плоскостью земли.
     Учитываются roll, pitch и yaw дрона.
  6. Публикация TargetDetection.
"""
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from swarm_interfaces.msg import TargetDetection


# HSV-диапазоны (OpenCV: H 0..179, S 0..255, V 0..255)
COLOR_RANGES = {
    "red":     [((0,   120, 70),  (10,  255, 255)),
                ((170, 120, 70),  (179, 255, 255))],
    "green":   [((40,  80,  60),  (85,  255, 255))],
    "blue":    [((100, 120, 60),  (130, 255, 255))],
    "yellow":  [((20,  120, 100), (35,  255, 255))],
    "magenta": [((140, 100, 80),  (170, 255, 255))],
}

MIN_AREA = 150
MAX_AREA = 50000


class TargetDetector(Node):
    def __init__(self):
        super().__init__("target_detector")

        self.declare_parameter("drone_id", 1)
        self.declare_parameter("fx", 381.36)
        self.declare_parameter("fy", 381.36)
        self.declare_parameter("cx", 320.5)
        self.declare_parameter("cy", 240.5)
        self.declare_parameter("min_altitude", 1.0)
        # Калибровка ориентации камеры относительно тела:
        self.declare_parameter("sign_u", 1.0)     # знак для (u - cx)
        self.declare_parameter("sign_v", -1.0)    # знак для (v - cy)
        self.declare_parameter("swap_uv", False)  # поменять u и v местами

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.min_alt = float(self.get_parameter("min_altitude").value)
        self.sign_u = float(self.get_parameter("sign_u").value)
        self.sign_v = float(self.get_parameter("sign_v").value)
        self.swap_uv = bool(self.get_parameter("swap_uv").value)

        self.bridge = CvBridge()
        self.height_agl = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Кватернион world<-body (w, x, y, z). Единичный = горизонтально, nose на север.
        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(
            Image,
            f"/camera_{self.drone_id}/downward_{self.drone_id}/image_raw",
            self.on_image, img_qos)

        self.create_subscription(
            VehicleLocalPosition,
            f"/px4_{self.drone_id}/fmu/out/vehicle_local_position",
            self.on_local_pos, px4_qos)

        self.create_subscription(
            VehicleAttitude,
            f"/px4_{self.drone_id}/fmu/out/vehicle_attitude",
            self.on_attitude, px4_qos)

        self.pub_det = self.create_publisher(
            TargetDetection, "/targets/detections", 10)

        self.pub_dbg = self.create_publisher(
            Image, f"/px4_{self.drone_id}/debug/detection_image", 1)

        self.get_logger().info(
            f"target_detector drone_id={self.drone_id}, "
            f"fx={self.fx:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}, "
            f"sign_u={self.sign_u:+.0f}, sign_v={self.sign_v:+.0f}, "
            f"swap_uv={self.swap_uv}")

    # ---------- callbacks ----------

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.height_agl = -msg.z  # в NED z<0 при подъёме

    def on_attitude(self, msg: VehicleAttitude):
        q = msg.q  # [w, x, y, z]
        self.q_w = float(q[0])
        self.q_x = float(q[1])
        self.q_y = float(q[2])
        self.q_z = float(q[3])

    def on_image(self, msg: Image):
        if self.height_agl < self.min_alt:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge fail: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dbg = frame.copy()

        # Матрица поворота world_from_body из текущего кватерниона (считаем один раз на кадр)
        qw, qx, qy, qz = self.q_w, self.q_x, self.q_y, self.q_z
        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r02 = 2*(qx*qz + qy*qw)
        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)
        r12 = 2*(qy*qz - qx*qw)
        r20 = 2*(qx*qz - qy*qw)
        r21 = 2*(qy*qz + qx*qw)
        r22 = 1 - 2*(qx*qx + qy*qy)

        for color, ranges in COLOR_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else (mask | m)

            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA or area > MAX_AREA:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                u = M["m10"] / M["m00"]
                v = M["m01"] / M["m00"]

                # --- Луч из пикселя в системе КАМЕРЫ (X вправо, Y вниз, Z вперёд/вниз) ---
                rx_cam = (u - self.cx) / self.fx
                ry_cam = (v - self.cy) / self.fy
                rz_cam = 1.0

                # --- Камера → BODY (FRD: X вперёд, Y вправо, Z вниз) ---
                # Нижняя камера: её Z_cam направлена вдоль +Z_body (вниз).
                # X_cam (вправо в кадре) → +Y_body (вправо у дрона).
                # Y_cam (вниз в кадре) → +X_body (вперёд у дрона).
                # sign_u/sign_v/swap_uv — поправки на возможные развороты SDF-камеры.
                du = self.sign_u * rx_cam
                dv = self.sign_v * ry_cam
                if self.swap_uv:
                    du, dv = dv, du
                rx_body = dv       # вперёд
                ry_body = du       # вправо
                rz_body = rz_cam   # вниз

                # --- BODY → WORLD (NED) полной матрицей поворота ---
                rx_w = r00*rx_body + r01*ry_body + r02*rz_body
                ry_w = r10*rx_body + r11*ry_body + r12*rz_body
                rz_w = r20*rx_body + r21*ry_body + r22*rz_body

                # --- Пересечение луча с плоскостью земли z=0 (в NED z вниз, земля = 0) ---
                # Позиция дрона в NED: (pos_x, pos_y, -height_agl).
                # P(t).z = -height_agl + t * rz_w = 0  =>  t = height_agl / rz_w.
                if rz_w < 1e-3:
                    # Луч почти горизонтален — проекция недостоверна, пропускаем.
                    continue
                t = self.height_agl / rz_w
                world_x = self.pos_x + t * rx_w
                world_y = self.pos_y + t * ry_w

                det = TargetDetection()
                det.agent_id = self.drone_id
                det.target_id = int(
                    (round(world_x * 2) * 100 + round(world_y * 2)) & 0xFF)
                det.world_position.x = float(world_x)
                det.world_position.y = float(world_y)
                det.world_position.z = 0.0
                det.color_label = color
                det.confidence = float(min(1.0, area / 2000.0))
                det.stamp = self.get_clock().now().to_msg()
                self.pub_det.publish(det)

                # визуализация
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(dbg, f"{color} ({world_x:+.1f},{world_y:+.1f})",
                            (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1)

        if self.pub_dbg.get_subscription_count() > 0:
            try:
                out = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                out.header = msg.header
                self.pub_dbg.publish(out)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()