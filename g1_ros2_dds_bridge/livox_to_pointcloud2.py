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

        dds_domain_id = int(self.get_parameter("dds_domain_id").value)
        dds_topic = str(self.get_parameter("dds_topic").value)
        ros_topic = str(self.get_parameter("ros_topic").value)
        self.override_frame_id = str(self.get_parameter("override_frame_id").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(PointCloud2, ros_topic, qos)

        ChannelFactoryInitialize(dds_domain_id)
        self.dds_sub = ChannelSubscriber(dds_topic, DdsPointCloud2_)
        self.dds_sub.Init(self._dds_cb)

        self.get_logger().info(f"Subscribed to DDS {dds_topic} (domain {dds_domain_id}), publishing {ros_topic}")

    def _dds_cb(self, msg: DdsPointCloud2_):
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

        cloud.fields = []
        for f in msg.fields:
            pf = PointField()
            pf.name = str(f.name)
            pf.offset = int(f.offset)
            pf.datatype = int(dds_pf_to_ros_datatype(int(f.datatype)))
            pf.count = int(f.count)
            cloud.fields.append(pf)

        cloud.data = bytes(msg.data)

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