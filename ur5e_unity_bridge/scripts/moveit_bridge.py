#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive

# Import service from ur5e_unity_bridge
from ur5e_unity_bridge.srv import MoveToPose

class MoveItBridgeNode(Node):
    def __init__(self):
        super().__init__('moveit_bridge_node')

        # Parameters
        self.group_name = self.declare_parameter('group_name', 'ur_manipulator').value
        self.end_effector_link = self.declare_parameter('end_effector_link', 'tool0').value
        self.reference_frame = self.declare_parameter('reference_frame', 'base_link').value

        # Action & Service Clients
        self.cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self.cartesian_path_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        # Service Server for Unity
        self.srv = self.create_service(MoveToPose, '/move_robot_to_pose', self.move_to_pose_callback, callback_group=self.cb_group)

        self.get_logger().info(f'MoveIt Bridge Node v3 [Robust] Initialized. Group: {self.group_name}')

    async def move_to_pose_callback(self, request, response):
        """Main service callback for moving the robot."""
        self.get_logger().info('--- New MoveToPose Request ---')
        target_pose = request.target_pose
        
        try:
            # 1. Attempt Cartesian Path (Straight Line)
            self.get_logger().info('[1/3] Calculating Cartesian Path...')
            cartesian_plan, fraction = await self.compute_cartesian_path(target_pose)
            
            if cartesian_plan and fraction >= 0.95:
                self.get_logger().info(f'✅ Straight path found (Fraction: {fraction:.2f}). Executing...')
                success = await self.execute_trajectory(cartesian_plan)
                if success:
                    response.success = True
                    response.message = f"Success: Cartesian ({fraction:.2f})"
                    return response
                else:
                    self.get_logger().warn('Cartesian execution failed. Trying fallback...')
            else:
                self.get_logger().warn(f'Straight path blocked (Fraction: {fraction:.2f}). Trying Fallback...')

            # 2. Fallback to Standard OMPL Planning
            self.get_logger().info('[2/3] Attempting Standard OMPL Planning...')
            success, msg = await self.standard_plan_and_execute(target_pose)
            
            response.success = success
            response.message = msg

        except Exception as e:
            self.get_logger().error(f'Unexpected error in callback: {str(e)}')
            response.success = False
            response.message = f"Internal Error: {str(e)}"
        
        self.get_logger().info('--- Request Finished ---')
        return response

    async def compute_cartesian_path(self, target_pose):
        """Calculates a linear path using MoveIt Service."""
        if not self.cartesian_path_client.wait_for_service(timeout_sec=3.0):
            return None, 0.0

        req = GetCartesianPath.Request()
        req.header.frame_id = self.reference_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.group_name
        req.link_name = self.end_effector_link
        req.waypoints = [target_pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        # Initialize start_state as empty and is_diff=True to use current state
        req.start_state.is_diff = True

        try:
            future = self.cartesian_path_client.call_async(req)
            result = await future # rclpy futures are awaitable in async handles
            return result.solution, result.fraction
        except Exception as e:
            self.get_logger().error(f'Cartesian service call failed: {e}')
            return None, 0.0

    async def execute_trajectory(self, trajectory):
        """Executes a pre-calculated trajectory."""
        if not self._execute_client.wait_for_server(timeout_sec=3.0):
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        try:
            goal_handle = await self._execute_client.send_goal_async(goal)
            
            if not goal_handle.accepted:
                return False

            result = await goal_handle.get_result_async()
            return result.result.error_code.val == MoveItErrorCodes.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Trajectory execution failed: {e}')
            return False

    async def standard_plan_and_execute(self, target_pose):
        """Standard Pick-and-Place style planning fallback."""
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return False, "MoveGroup server not ready"

        goal = MoveGroup.Goal()
        goal.request = self.create_motion_plan_request(target_pose)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        
        try:
            self.get_logger().info('Sending MoveGroup goal...')
            goal_handle = await self._action_client.send_goal_async(goal)
            
            if not goal_handle.accepted:
                return False, "Goal Rejected"

            result = await goal_handle.get_result_async()
            
            err = result.result.error_code.val
            if err == MoveItErrorCodes.SUCCESS:
                return True, "Success: Standard Planning"
            else:
                return False, f"Failed: MoveIt Error {err}"
        except Exception as e:
            return False, f"Execution Error: {str(e)}"

    def create_motion_plan_request(self, target_pose):
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = self.reference_frame
        req.start_state.is_diff = True
        req.group_name = self.group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.reference_frame
        pc.link_name = self.end_effector_link
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header.frame_id = self.reference_frame
        oc.link_name = self.end_effector_link
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        return req

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridgeNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
