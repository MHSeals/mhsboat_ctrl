#!/usr/bin/env python
# publishes center of clusters in a PointCloud2
import rclpy
from rclpy.node import Node
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from sklearn.cluster import DBSCAN
import traceback

from mhsboat_ctrl.utils.lidar import read_points


class center_of_clusters(Node):
    def __init__(self):
        super().__init__("center_of_clusters")

        self.get_logger().info("Starting")

        self.pcd_publisher = self.create_publisher(
            PointCloud2, "center_of_clusters", 10
        )
        # self.subscription = self.create_subscription(PointCloud2,
        # "/wamv/sensors/lidars/lidar_wamv_sensor/points",self.listener_callback,10
        # )

        # REAL TOPIC
        self.subscription = self.create_subscription(
            PointCloud2, "/velodyne_points", self.listener_callback, 10
        )

        self.allPointsPublisher = self.create_publisher(
            PointCloud2, "/all_points", 10)
        
        self.pcd = PointCloud2()

        self.get_logger().info("Started")

    def listener_callback(self, msg: PointCloud2):
        self.get_logger().info("Processing new point cloud")
        self.pcd = safe_point_cloud(msg, self)
        self.get_logger().info("Publishing center of clusters")
        self.pcd_publisher.publish(self.pcd)

    def mypublishAllPoints(self, data: PointCloud2):
        self.get_logger().info("Publishing all points")
        self.allPointsPublisher.publish(data)

def safe_point_cloud(msg: PointCloud2, node: center_of_clusters) -> PointCloud2:
    """
    Wraps the point_cloud function in a try/except block to catch any errors

    Args:
        msg (PointCloud2): PointCloud2 message
        node (center_of_clusters): Node that will publish the point cloud

    Returns:
        PointCloud2: _description_
    """
    try:
        return point_cloud(msg, node)
    except Exception as e:
        print("Error processing point cloud: ")
        traceback.print_exc()
        return msg

def point_cloud(msg: PointCloud2, node: center_of_clusters) -> PointCloud2:
    """
    Converts a ROS PointCloud2 message to a np array

    :param msg: A sensor_msgs.PointCloud2 message
    :type msg: class:`sensor_msgs.msg.PointCloud2`
    :param node: The node that will publish the point cloud
    :type node: class:`center_of_clusters`
    :return: list of points
    :rtype: class:`sensor_msgs.msg.PointCloud2`
    """

    points = np.array(list(read_points(msg)))

    # get red of infinite points
    # print("before: "+str(len(points)))
    mask = np.isinf(points).any(axis=1)
    points = points[~mask]

    points = points[:, 0:3]

    # get rid of points above the lidar
    # can be configured in hardware rather than software for the actual lidar
    # high_points_mask = points[:, 2] < 0
    # points = points[high_points_mask]
    # print("after: "+str(len(points)))

    # cluster the points
    db = DBSCAN(eps=0.09, min_samples=10).fit(points)
    labels = db.labels_
    max = np.max(labels)

    # get rid of unclustered points
    points = np.delete(points, np.where(labels == -1), axis=0)
    labels = np.delete(labels, np.where(labels == -1))

    # print("max: "+str(max))
    # print(len(points))
    # print(len(labels))

    # calculate center of each cluster
    clusterCenters = np.zeros((max, points.shape[1]))
    for i in range(max):
        indices = np.where(labels == i)[0]
        clusterPoints = points[indices]
        clusterCenters[i] = np.mean(clusterPoints, axis=0)

    # print("clusterCenters: "+str(clusterCenters))

    clusterCenters = np.delete(
        clusterCenters, np.where(clusterCenters == [np.inf]), axis=0
    )
    print("clusterCenters length: " + str(len(clusterCenters)))

    # print("clusterCenters: "+str(clusterCenters))

    colors = np.random.randint(255, size=(max + 2, 3))
    colors[0] = [0, 0, 0]

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgba", offset=12,
                   datatype=PointField.UINT32, count=1),
    ]

    header = Header()
    header.frame_id = "map"

    point_struct = struct.Struct("<fffBBBB")

    buffer = bytearray(point_struct.size * len(clusterCenters))

    buffer2 = bytearray(point_struct.size * len(points))
    for i, point in enumerate(points):
        point_struct.pack_into(
            buffer2,
            i * point_struct.size,
            point[0],
            point[1],
            point[2],
            colors[labels[i]][0],
            colors[labels[i]][1],
            colors[labels[i]][2],
            255,
        )

    allPoints = PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(16),
        row_step=(24 * points.shape[0]),
        data=buffer2,
    )
    print(type(allPoints))
    node.mypublishAllPoints(allPoints)

    for i, point in enumerate(clusterCenters):
        point_struct.pack_into(
            buffer, i *
            point_struct.size, point[0], point[1], point[2], 0, 0, 0, 255
        )
    return PointCloud2(
        header=header,
        height=1,
        width=clusterCenters.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(16),
        row_step=(24 * clusterCenters.shape[0]),
        data=buffer,
    )


def main(args=None):
    rclpy.init(args=args)
    publisher = center_of_clusters()

    try:
        rclpy.spin(publisher)
        publisher.get_logger().info("Starting")
    except KeyboardInterrupt:
        publisher.get_logger().info("Shutting down")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
