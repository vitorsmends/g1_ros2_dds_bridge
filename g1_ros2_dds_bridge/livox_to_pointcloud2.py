#!/usr/bin/env python3
import math
import struct
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock, ClockType

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


def compute_point_step(fields: List[PointField]) -> int:
    # Minimal point_step that can hold all fields at their declared offsets
    ps = 0
    for f in fields:
        ps = max(ps, int(f.offset) + sizeof_ros_datatype(int(f.datatype)) * int(f.count))
    return int(ps)


def find_field(fields: List[PointField], name: str) -> PointField | None:
    for f in fields:
        if f.name == name:
            return f
    return None


def hexdump_prefix(b: bytes, n: int = 32) -> str:
    p = b[:n]
    return " ".join(f"{x:02x}" for x in p)


def unpack_f32(data: bytes, offset: int, endian: str) -> float:
    # endian: "<" little, ">" big
    return struct.unpack_from(endian + "f", data, offset)[0]


class LivoxToPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_to_pointcloud2")

        # Core params
        self.declare_parameter("dds_domain_id", 0)
        self.declare_parameter("dds_topic", "rt/utlidar/cloud_livox_mid360")
        self.declare_parameter("ros_topic", "/livox/lidar")
        self.declare_parameter("override_frame_id", "")
        self.declare_parameter("dds_queue_depth", 32)
        self.declare_parameter("log_every_n", 30)

        # Fixes / safety
        self.declare_parameter("force_little_endian", True)
        self.declare_parameter("recompute_steps", True)
        self.declare_parameter("sanitize_frame_id", True)

        # Stamp handling (important for your case: use_sim_time True + no /clock => stamp=0)
        # - prefer_system_time_stamp=True forces non-zero stamps even if /clock is absent.
        self.declare_parameter("prefer_system_time_stamp", True)
        self.declare_parameter("warn_if_stamp_zero", True)

        # Debug / validation
        self.declare_parameter("debug_first_point_xyz", True)
        self.declare_parameter("debug_sample_points", 5)              # how many points to sample for stats (0 disables)
        self.declare_parameter("debug_check_both_endians", True)       # decode xyz with <f and >f to detect endian mismatch
        self.declare_parameter("debug_dump_data_prefix_bytes", 48)     # hex dump prefix (0 disables)
        self.declare_parameter("debug_warn_if_all_invalid", True)      # warn if sampled points are all non-finite

        # Read params
        dds_domain_id = int(self.get_parameter("dds_domain_id").value)
        dds_topic = str(self.get_parameter("dds_topic").value)
        ros_topic = str(self.get_parameter("ros_topic").value)
        self.override_frame_id = str(self.get_parameter("override_frame_id").value)
        self.dds_queue_depth = int(self.get_parameter("dds_queue_depth").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        self.force_little_endian = bool(self.get_parameter("force_little_endian").value)
        self.recompute_steps = bool(self.get_parameter("recompute_steps").value)
        self.sanitize_frame_id = bool(self.get_parameter("sanitize_frame_id").value)

        self.prefer_system_time_stamp = bool(self.get_parameter("prefer_system_time_stamp").value)
        self.warn_if_stamp_zero = bool(self.get_parameter("warn_if_stamp_zero").value)

        self.debug_first_point_xyz = bool(self.get_parameter("debug_first_point_xyz").value)
        self.debug_sample_points = int(self.get_parameter("debug_sample_points").value)
        self.debug_check_both_endians = bool(self.get_parameter("debug_check_both_endians").value)
        self.debug_dump_data_prefix_bytes = int(self.get_parameter("debug_dump_data_prefix_bytes").value)
        self.debug_warn_if_all_invalid = bool(self.get_parameter("debug_warn_if_all_invalid").value)

        # Publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(PointCloud2, ros_topic, qos)

        # DDS
        ChannelFactoryInitialize(dds_domain_id)
        self.dds_sub = ChannelSubscriber(dds_topic, DdsPointCloud2_)
        try:
            self.dds_sub.Init(self._dds_cb, self.dds_queue_depth)
        except TypeError:
            self.dds_sub.Init(self._dds_cb)

        # Clocks
        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # Counters
        self.msg_count = 0
        self.bad_layout_count = 0
        self.invalid_xyz_count = 0
        self.zero_stamp_count = 0
        self.all_invalid_sample_count = 0

        self.get_logger().info(
            f"Subscribed to DDS '{dds_topic}' (domain {dds_domain_id}), publishing '{ros_topic}'. "
            f"prefer_system_time_stamp={self.prefer_system_time_stamp}, "
            f"force_little_endian={self.force_little_endian}, recompute_steps={self.recompute_steps}"
        )

    def _stamp_now(self):
        # If use_sim_time is true and /clock isn't publishing, self.get_clock().now() stays at 0.
        # This forces system time to avoid stamp=0 (which breaks TF message_filters and other consumers).
        if self.prefer_system_time_stamp:
            return self.sys_clock.now().to_msg()
        return self.get_clock().now().to_msg()

    def _dds_cb(self, msg: DdsPointCloud2_):
        self.msg_count += 1

        cloud = PointCloud2()
        cloud.header.stamp = self._stamp_now()

        if cloud.header.stamp.sec == 0 and cloud.header.stamp.nanosec == 0:
            self.zero_stamp_count += 1
            if self.warn_if_stamp_zero:
                self.get_logger().warn(
                    f"Publishing PointCloud2 with ZERO stamp (use_sim_time likely enabled but /clock absent). "
                    f"zero_stamp_total={self.zero_stamp_count}"
                )

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
        fields: List[PointField] = []
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

        # Endianness
        dds_is_bigendian = bool(msg.is_bigendian)
        cloud.is_bigendian = False if self.force_little_endian else dds_is_bigendian

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

        # ---- XYZ decode + validity stats ----
        # Determine xyz fields (and validate they are float32, count=1)
        fx = find_field(fields, "x")
        fy = find_field(fields, "y")
        fz = find_field(fields, "z")
        has_xyz = (fx is not None and fy is not None and fz is not None)

        # Decode first point xyz in little-endian (and optionally big-endian) to detect mismatch
        x = y = z = float("nan")
        xyz_ok = False
        endian_used = "<"
        endian_alt_ok = None  # None/True/False
        x_be = y_be = z_be = float("nan")

        if has_xyz and actual_len >= cloud.point_step and cloud.point_step > 0:
            try:
                x = unpack_f32(data_bytes, int(fx.offset), "<")
                y = unpack_f32(data_bytes, int(fy.offset), "<")
                z = unpack_f32(data_bytes, int(fz.offset), "<")
                xyz_ok = (math.isfinite(x) and math.isfinite(y) and math.isfinite(z))
                endian_used = "<"
            except Exception:
                xyz_ok = False

            if self.debug_check_both_endians:
                try:
                    x_be = unpack_f32(data_bytes, int(fx.offset), ">")
                    y_be = unpack_f32(data_bytes, int(fy.offset), ">")
                    z_be = unpack_f32(data_bytes, int(fz.offset), ">")
                    endian_alt_ok = (math.isfinite(x_be) and math.isfinite(y_be) and math.isfinite(z_be))
                except Exception:
                    endian_alt_ok = False

        if has_xyz and not xyz_ok:
            self.invalid_xyz_count += 1

        # Sample a few points to estimate %finite
        sampled = 0
        finite = 0
        finite_xy = 0
        finite_xyz = 0
        max_points = cloud.width * cloud.height if cloud.point_step > 0 else 0

        if has_xyz and self.debug_sample_points > 0 and max_points > 0:
            # choose indices roughly spread: 0, mid, last, plus a couple more if available
            idxs = [0]
            if max_points > 2:
                idxs.append(max_points // 2)
            if max_points > 1:
                idxs.append(max_points - 1)
            # fill remaining with small indices
            i = 1
            while len(idxs) < min(self.debug_sample_points, max_points):
                if i not in idxs:
                    idxs.append(i)
                i += 1

            for idx in idxs:
                base = idx * cloud.point_step
                if base + cloud.point_step > actual_len:
                    continue
                try:
                    sx = unpack_f32(data_bytes, base + int(fx.offset), "<")
                    sy = unpack_f32(data_bytes, base + int(fy.offset), "<")
                    sz = unpack_f32(data_bytes, base + int(fz.offset), "<")
                    sampled += 1
                    if math.isfinite(sx) and math.isfinite(sy):
                        finite_xy += 1
                    if math.isfinite(sx) and math.isfinite(sy) and math.isfinite(sz):
                        finite_xyz += 1
                except Exception:
                    sampled += 1

            if sampled > 0:
                finite = finite_xyz

            if sampled > 0 and finite_xyz == 0:
                self.all_invalid_sample_count += 1

        # ---- Logging ----
        if self.log_every_n > 0 and (self.msg_count % self.log_every_n) == 0:
            stamp = cloud.header.stamp
            stamp_s = f"{stamp.sec}.{stamp.nanosec:09d}"

            dump = ""
            if self.debug_dump_data_prefix_bytes and self.debug_dump_data_prefix_bytes > 0:
                dump = hexdump_prefix(data_bytes, self.debug_dump_data_prefix_bytes)

            endian_note = ""
            if self.debug_check_both_endians and has_xyz:
                endian_note = (
                    f" endianCheck=LE_ok:{xyz_ok} BE_ok:{endian_alt_ok} "
                    f"LE_xyz=({x:.3f},{y:.3f},{z:.3f}) "
                    f"BE_xyz=({x_be:.3f},{y_be:.3f},{z_be:.3f})"
                )

            sample_note = ""
            if has_xyz and self.debug_sample_points > 0 and sampled > 0:
                sample_note = (
                    f" sample(n={sampled}) finite_xy={finite_xy}/{sampled} "
                    f"finite_xyz={finite_xyz}/{sampled} "
                    f"finite_xyz_pct={(finite_xyz/sampled*100.0):.1f}%"
                )

            self.get_logger().info(
                f"[DDS->ROS] msgs={self.msg_count} stamp={stamp_s} frame_id='{cloud.header.frame_id}' "
                f"hw=({cloud.height},{cloud.width}) "
                f"dds_steps=(ps={dds_point_step}, rs={dds_row_step}) "
                f"ros_steps=(ps={cloud.point_step}, rs={cloud.row_step}) "
                f"endian(dds={dds_is_bigendian}, ros={cloud.is_bigendian}) "
                f"data_len={actual_len} expected_len={expected_len} layout_ok={layout_ok} "
                f"fields={len(fields)} has_xyz={has_xyz} is_dense={cloud.is_dense} "
                f"firstLE_xyz=({x:.3f},{y:.3f},{z:.3f}) firstLE_ok={xyz_ok} "
                f"bad_layout_total={self.bad_layout_count} invalid_xyz_total={self.invalid_xyz_count} "
                f"zero_stamp_total={self.zero_stamp_count} all_invalid_sample_total={self.all_invalid_sample_count}"
                + endian_note
                + sample_note
                + (f" data_prefix[{self.debug_dump_data_prefix_bytes}B]={dump}" if dump else "")
            )

        # Targeted warnings (only when meaningful)
        if not has_xyz:
            self.get_logger().warn("PointCloud2 missing one or more required fields: x, y, z")

        if has_xyz and self.debug_warn_if_all_invalid and sampled > 0 and finite_xyz == 0:
            self.get_logger().warn(
                "Sampled points appear ALL non-finite (x/y/z). This strongly indicates endian/offset/step mismatch "
                "or invalid payload coming from DDS."
            )

        if not layout_ok and expected_len > 0:
            self.get_logger().warn(
                f"PointCloud2 layout mismatch: data_len={actual_len} expected_len={expected_len} "
                f"(row_step*height). Likely wrong steps or payload size mismatch."
            )

        if has_xyz and not xyz_ok and self.debug_first_point_xyz:
            self.get_logger().warn(
                f"First point LE xyz non-finite: x={x} y={y} z={z}. "
                f"If BE decodes finite (BE_ok=True), your endianness flag/interpretation is wrong."
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