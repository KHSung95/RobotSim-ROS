#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct

from ur5e_unity_bridge.srv import CalculateICP

class ICPServiceServer(Node):

    def __init__(self):
        super().__init__('icp_server')
        self.srv = self.create_service(CalculateICP, 'calculate_icp', self.calculate_icp_callback)
        self.get_logger().info('ICP Service Server Ready (Robust: Coarse-to-Fine).')

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

    def calculate_icp_callback(self, request, response):
        self.get_logger().info('--- Received ICP Request ---')

        # 1. Convert Clouds
        target = self.convert_ros_to_o3d(request.master_point_cloud) 
        source = self.convert_ros_to_o3d(request.current_point_cloud)

        if target is None or source is None:
            self.get_logger().error("One of the point clouds is empty or invalid.")
            response.success = False
            response.transformation_matrix = np.eye(4, dtype=np.float32).flatten().tolist()
            return response

        if len(target.points) < 10 or len(source.points) < 10:
             self.get_logger().warn(f"Point clouds too small. Source: {len(source.points)}, Target: {len(target.points)}")
             response.success = False
             response.transformation_matrix = np.eye(4, dtype=np.float32).flatten().tolist()
             return response

        # 2. Data Inspection (Centroids & Scale)
        source_center = source.get_center()
        target_center = target.get_center()
        dist = np.linalg.norm(source_center - target_center)
        
        self.get_logger().info(f"Source Center: {source_center}")
        self.get_logger().info(f"Target Center: {target_center}")
        self.get_logger().info(f"Centroid Distance: {dist:.4f} m")

        # 3. Pre-processing
        voxel_size = 0.005 # 5mm
        # Create downsampled copies for registration
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)

        # Estimate Normals
        radius_normal = voxel_size * 4 # 0.02m (Robust Normal Estimation)
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        # 4. Initialization Heuristic
        # Instead of identity, align centroids as initial guess
        trans_init = np.eye(4)
        trans_init[:3, 3] = target_center - source_center
        self.get_logger().info(f"Initial Align Guess (Translation):\n{trans_init[:3,3]}")

        # 5. Two-Pass ICP
        
        # --- Pass 1: Coarse (Point-to-Point, Large Threshold) ---
        coarse_threshold = 1.0 # 1 meter tolerance
        self.get_logger().info("Starting Pass 1 (Coarse)...")
        reg_coarse = o3d.pipelines.registration.registration_icp(
            source_down, target_down, coarse_threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        self.get_logger().info(f"Pass 1 Fitness: {reg_coarse.fitness:.4f}")

        # --- Pass 2: Fine (Point-to-Plane, Small Threshold) ---
        fine_threshold = 0.05 # 5cm tolerance for final snap
        self.get_logger().info("Starting Pass 2 (Fine)...")
        
        # Use result of Pass 1 as init for Pass 2
        try:
            reg_fine = o3d.pipelines.registration.registration_icp(
                source_down, target_down, fine_threshold, reg_coarse.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
            )
        except Exception as e:
            self.get_logger().warn(f"Point-to-Plane failed ({e}), keeping Coarse result.")
            reg_fine = reg_coarse

        # 6. Result
        final_transform = reg_fine.transformation
        fitness = reg_fine.fitness
        rmse = reg_fine.inlier_rmse

        self.get_logger().info(f"Final ICP Converged. Fitness: {fitness:.4f}, RMSE: {rmse:.4f}")
        # self.get_logger().info(f"Transform:\n{final_transform}")

        response.transformation_matrix = final_transform.flatten().tolist()
        response.fitness = float(fitness)
        # Consider successful if some overlap found, or if coarse was good enough
        response.success = fitness > 0.0 or reg_coarse.fitness > 0.0
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ICPServiceServer()
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
