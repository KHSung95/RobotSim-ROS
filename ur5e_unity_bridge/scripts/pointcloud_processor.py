#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct

from ur5e_unity_bridge.srv import ProcessScene

class PointCloudProcessor(Node):

    def __init__(self):
        super().__init__('pointcloud_processor')
        self.srv = self.create_service(ProcessScene, 'process_scene', self.process_scene_callback)
        self.master_pcd = None
        self.get_logger().info('PointCloud Processor Ready.')

    def convert_ros_to_o3d(self, ros_cloud):
        """Converts sensor_msgs/PointCloud2 to open3d.geometry.PointCloud"""
        try:
            # Efficient conversion
            # Use skip_nans=True to filter bad points
            gen = pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z"))
            cloud_data = list(gen)
            
            if not cloud_data:
                return None

            # Convert directly to float64 numpy array. 
            if len(cloud_data) > 0 and not isinstance(cloud_data[0], (list, tuple, np.ndarray)):
                 cloud_data = [list(p) for p in cloud_data]

            xyz = np.array(cloud_data, dtype=np.float64)
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            return pcd
            
        except Exception as e:
            self.get_logger().error(f"Error converting PointCloud: {e}")
            return None

    def process_scene_callback(self, request, response):
        cmd = request.command.upper()
        self.get_logger().info(f"Received Command: {cmd}")

        if cmd == "SET_MASTER":
             if request.input_cloud.width * request.input_cloud.height == 0:
                 self.get_logger().warn("Empty cloud received for SET_MASTER")
                 response.success = False
                 return response
                 
             pcd = self.convert_ros_to_o3d(request.input_cloud)
             if pcd:
                 self.master_pcd = pcd
                 response.success = True
                 self.get_logger().info(f"Master Set: {len(pcd.points)} points")
             else:
                 response.success = False
                 self.get_logger().error("Failed to convert Master Cloud")
             return response

        elif cmd == "COMPARE":
             if self.master_pcd is None:
                 self.get_logger().warn("No Master Cloud Set.")
                 response.success = False
                 return response
             
             current_pcd = self.convert_ros_to_o3d(request.input_cloud)
             if not current_pcd:
                 response.success = False
                 return response
                 
             # Compute Distance (KD-Tree)
             # compute_point_cloud_distance returns a DoubleVector
             dists = current_pcd.compute_point_cloud_distance(self.master_pcd)
             dists = np.asarray(dists)
             
             # Calculate Match Status (Intensity)
             threshold = request.threshold
             # 1.0 = Match, 0.0 = Mismatch
             intensities = np.where(dists <= threshold, 1.0, 0.0).astype(np.float32)
             
             match_score = np.mean(intensities)
             self.get_logger().info(f"Match Score: {match_score:.4f} (Threshold: {threshold})")
             
             # Create Result Cloud
             # XYZ from current_pcd (float64 -> float32 for PC2)
             xyz = np.asarray(current_pcd.points).astype(np.float32)
             
             # Stack [x, y, z, intensity]
             points_data = np.hstack((xyz, intensities.reshape(-1, 1)))
             
             # Define fields for PointCloud2 (x, y, z, intensity)
             fields = [
                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                 PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
             ]
             
             # Prepare Header
             header = request.input_cloud.header
             
             # Create PointCloud2
             result_cloud = pc2.create_cloud(header, fields, points_data)
             
             response.success = True
             response.match_score = float(match_score)
             response.result_cloud = result_cloud
             response.correction_matrix = np.eye(4, dtype=np.float32).flatten().tolist() # Placeholder
             
             return response
             
        else:
            self.get_logger().warn(f"Unknown Command: {cmd}")
            response.success = False
            return response

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
