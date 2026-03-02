#!/usr/bin/env python3
import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_ as DdsPointCloud2_


def dds_pf_to_ros_datatype(dds_datatype: int) -> int:
    m = {
        1: PointField.INT8,
        2: PointField.UINT8,
        3: PointField.INT16,
        4: PointField.UINT16,
        5: PointField.INT32,
        6: PointField.UINT32,
        7: PointField.FLOAT32,
        8: PointField.FLOAT64,
    }
    return m.get(int(dds_datatype), PointField.FLOAT32)


def sizeof_ros_datatype(dt: int) -> int:
    return {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }.get(int(dt), 4)


def compute_point_step(fields: list[PointField]) -> int:
    # Minimal point_step that can hold all fields at their declared offsets
    ps = 0
    for f in fields:
        ps = max(ps, int(f.offset) + sizeof_ros_datatype(int(f.datatype)) * int(f.count))
    return int(ps)


class LivoxToPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_to_pointcloud2")

        self.declare_parameter("dds_domain_id", 0)
        self.declare_parameter("dds_topic", "rt/utlidar/cloud_livox_mid360")
        self.declare_parameter("ros_topic", "/livox/lidar")
        self.declare_parameter("override_frame_id", "")
        self.declare_parameter("dds_queue_depth", 32)
        self.declare_parameter("log_every_n", 30)

        # Safety / fixes
        self.declare_parameter("force_little_endian", True)
        self.declare_parameter("recompute_steps", True)
        self.declare_parameter("sanitize_frame_id", True)

        # Debug
        self.declare_parameter("debug_first_point_xyz", True)

        dds_domain_id = int(self.get_parameter("dds_domain_id").value)
        dds_topic = str(self.get_parameter("dds_topic").value)
        ros_topic = str(self.get_parameter("ros_topic").value)
        self.override_frame_id = str(self.get_parameter("override_frame_id").value)
        self.dds_queue_depth = int(self.get_parameter("dds_queue_depth").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        self.force_little_endian = bool(self.get_parameter("force_little_endian").value)
        self.recompute_steps = bool(self.get_parameter("recompute_steps").value)
        self.sanitize_frame_id = bool(self.get_parameter("sanitize_frame_id").value)
        self.debug_first_point_xyz = bool(self.get_parameter("debug_first_point_xyz").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(PointCloud2, ros_topic, qos)

        ChannelFactoryInitialize(dds_domain_id)
        self.dds_sub = ChannelSubscriber(dds_topic, DdsPointCloud2_)
        try:
            self.dds_sub.Init(self._dds_cb, self.dds_queue_depth)
        except TypeError:
            self.dds_sub.Init(self._dds_cb)

        self.msg_count = 0
        self.bad_layout_count = 0
        self.invalid_xyz_count = 0

        self.get_logger().info(
            f"Subscribed to DDS '{dds_topic}' (domain {dds_domain_id}), publishing '{ros_topic}'"
        )

    def _dds_cb(self, msg: DdsPointCloud2_):
        self.msg_count += 1

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()

        fid = str(msg.header.frame_id)
        if self.sanitize_frame_id:
            fid = fid.strip("\x00").strip()

        cloud.header.frame_id = self.override_frame_id if self.override_frame_id else fid

        # Height/width sanity: for LiDAR it's typically height=1
        h = int(msg.height)
        w = int(msg.width)
        if h <= 0:
            h = 1
        if w < 0:
            w = 0
        cloud.height = h
        cloud.width = w

        # Build fields
        fields: list[PointField] = []
        for f in msg.fields:
            pf = PointField()
            pf.name = str(f.name).strip("\x00").strip()
            pf.offset = int(f.offset)
            pf.datatype = int(dds_pf_to_ros_datatype(int(f.datatype)))
            pf.count = int(f.count)
            fields.append(pf)
        cloud.fields = fields

        data_bytes = bytes(msg.data)
        cloud.data = data_bytes

        # Endianness: most ROS2 setups are little-endian; DDS flag is often unreliable
        cloud.is_bigendian = False if self.force_little_endian else bool(msg.is_bigendian)

        # Steps: recompute robustly, because DDS producers often send wrong row_step/point_step
        dds_point_step = int(msg.point_step)
        dds_row_step = int(msg.row_step)

        if self.recompute_steps and len(fields) > 0 and cloud.width > 0:
            ps = compute_point_step(fields)
            cloud.point_step = ps if ps > 0 else max(dds_point_step, 0)
            cloud.row_step = cloud.point_step * cloud.width
        else:
            cloud.point_step = max(dds_point_step, 0)
            cloud.row_step = max(dds_row_step, 0)

        cloud.is_dense = bool(msg.is_dense)

        expected_len = cloud.row_step * cloud.height if cloud.row_step > 0 else 0
        actual_len = len(data_bytes)

        layout_ok = True
        if expected_len > 0 and actual_len != expected_len:
            layout_ok = False
            self.bad_layout_count += 1

        # Verify XYZ by decoding first point using declared offsets
        has_xyz = {f.name for f in fields} >= {"x", "y", "z"}
        x = y = z = float("nan")
        xyz_ok = False
        if has_xyz and actual_len >= cloud.point_step and cloud.point_step > 0:
            off = {pf.name: pf.offset for pf in fields}
            try:
                # Interpret as little-endian float32 by default
                x = struct.unpack_from("<f", data_bytes, off["x"])[0]
                y = struct.unpack_from("<f", data_bytes, off["y"])[0]
                z = struct.unpack_from("<f", data_bytes, off["z"])[0]
                xyz_ok = (math.isfinite(x) and math.isfinite(y) and math.isfinite(z))
            except Exception:
                xyz_ok = False

        if has_xyz and not xyz_ok:
            self.invalid_xyz_count += 1

        if self.log_every_n > 0 and (self.msg_count % self.log_every_n) == 0:
            self.get_logger().info(
                f"[DDS->ROS] msgs={self.msg_count} frame_id='{cloud.header.frame_id}' "
                f"hw=({cloud.height},{cloud.width}) "
                f"dds_steps=(ps={dds_point_step}, rs={dds_row_step}) "
                f"ros_steps=(ps={cloud.point_step}, rs={cloud.row_step}) "
                f"data_len={actual_len} expected_len={expected_len} layout_ok={layout_ok} "
                f"fields={len(fields)} has_xyz={has_xyz} is_dense={cloud.is_dense} "
                f"first_xyz=({x:.3f},{y:.3f},{z:.3f}) xyz_ok={xyz_ok} "
                f"bad_layout_total={self.bad_layout_count} invalid_xyz_total={self.invalid_xyz_count}"
            )

        if not has_xyz:
            self.get_logger().warn("PointCloud2 missing one or more required fields: x, y, z")

        if not layout_ok and expected_len > 0:
            self.get_logger().warn(
                f"PointCloud2 layout mismatch: data_len={actual_len} expected_len={expected_len} "
                f"(row_step*height). Likely wrong steps from DDS or wrong payload size."
            )

        if has_xyz and not xyz_ok and self.debug_first_point_xyz:
            self.get_logger().warn(
                f"First point xyz decoded as non-finite: x={x} y={y} z={z}. "
                f"Likely endian/offset/step mismatch."
            )

        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = LivoxToPointCloud2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()