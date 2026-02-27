#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time as RosTime

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_ as DdsPointCloud2_


def dds_time_to_ros(sec: int, nanosec: int) -> RosTime:
    t = RosTime()
    t.sec = int(sec)
    t.nanosec = int(nanosec)
    return t


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


class LivoxToPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_to_pointcloud2")

        self.declare_parameter("dds_domain_id", 0)
        self.declare_parameter("dds_topic", "rt/utlidar/cloud_livox_mid360")
        self.declare_parameter("ros_topic", "/livox/lidar")
        self.declare_parameter("override_frame_id", "")
        self.declare_parameter("dds_queue_depth", 32)
        self.declare_parameter("log_every_n", 30)

        dds_domain_id = int(self.get_parameter("dds_domain_id").value)
        dds_topic = str(self.get_parameter("dds_topic").value)
        ros_topic = str(self.get_parameter("ros_topic").value)
        self.override_frame_id = str(self.get_parameter("override_frame_id").value)
        self.dds_queue_depth = int(self.get_parameter("dds_queue_depth").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
        self.all_inf_hint_count = 0

        self.get_logger().info(
            f"Subscribed to DDS {dds_topic} (domain {dds_domain_id}), publishing {ros_topic} "
            f"(dds_queue_depth={self.dds_queue_depth})"
        )

    def _dds_cb(self, msg: DdsPointCloud2_):
        self.msg_count += 1

        cloud = PointCloud2()
        cloud.header.stamp = dds_time_to_ros(msg.header.stamp.sec, msg.header.stamp.nanosec)

        fid = str(msg.header.frame_id)
        cloud.header.frame_id = self.override_frame_id if self.override_frame_id else fid

        cloud.height = int(msg.height)
        cloud.width = int(msg.width)
        cloud.is_bigendian = bool(msg.is_bigendian)
        cloud.point_step = int(msg.point_step)
        cloud.row_step = int(msg.row_step)
        cloud.is_dense = bool(msg.is_dense)

        fields = []
        for f in msg.fields:
            pf = PointField()
            pf.name = str(f.name)
            pf.offset = int(f.offset)
            pf.datatype = int(dds_pf_to_ros_datatype(int(f.datatype)))
            pf.count = int(f.count)
            fields.append(pf)
        cloud.fields = fields

        data_bytes = bytes(msg.data)
        cloud.data = data_bytes

        expected_len = cloud.row_step * cloud.height if cloud.height > 0 else 0
        actual_len = len(data_bytes)

        layout_ok = True
        if expected_len != 0 and actual_len != expected_len:
            layout_ok = False
            self.bad_layout_count += 1

        has_xyz = {f.name for f in fields} >= {"x", "y", "z"}
        all_inf_hint = False
        if actual_len >= 12:
            head = data_bytes[:12]
            if head == b"\x00\x00\x80\x7f" * 3:
                all_inf_hint = True
                self.all_inf_hint_count += 1

        if self.log_every_n > 0 and (self.msg_count % self.log_every_n) == 0:
            self.get_logger().info(
                f"[DDS->ROS] msgs={self.msg_count} frame_id='{cloud.header.frame_id}' "
                f"hw=({cloud.height},{cloud.width}) point_step={cloud.point_step} row_step={cloud.row_step} "
                f"data_len={actual_len} expected_len={expected_len} layout_ok={layout_ok} "
                f"fields={len(fields)} has_xyz={has_xyz} is_dense={cloud.is_dense} "
                f"all_inf_hint={all_inf_hint} bad_layout_total={self.bad_layout_count} all_inf_hint_total={self.all_inf_hint_count}"
            )

        if not has_xyz:
            self.get_logger().warn("PointCloud2 missing one or more of required fields: x, y, z")

        if not layout_ok:
            self.get_logger().warn(
                f"PointCloud2 layout mismatch: data_len={actual_len} expected_len={expected_len} "
                f"(row_step*height). Payload may be invalid."
            )

        if all_inf_hint:
            self.get_logger().warn(
                "PointCloud2 payload appears to start with FLOAT32 +inf pattern (00 00 80 7F). "
                "This usually indicates 'no-hit' rays are being published as inf."
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