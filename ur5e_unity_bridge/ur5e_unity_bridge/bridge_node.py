#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

class UR5eUnityBridge(Node):
    def __init__(self):
        super().__init__('ur5e_unity_bridge')
        
        # 1. Heartbeat Publisher
        self.health_pub = self.create_publisher(String, '/unity/health', 10)
        
        # [NEW] Unity 전용 JointState Publisher (NaN 제거 버전)
        self.unity_joint_pub = self.create_publisher(JointState, '/unity/joint_states', 10)
        
        # [NEW] 원본 JointState Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.heartbeat_count = 0
        
        # 2. Unity로부터 Target Pose를 받는 Subscriber
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/unity/target_pose',
            self.target_pose_callback,
            10
        )

        # 3. Unity로부터 직접 조인트 명령을 받는 Subscriber (에러 해결 및 모니터링용)
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/joint_command',
            self.joint_command_callback,
            10
        )
        
        # 4. MoveIt2의 move_group 액션 클라이언트
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info("==========================================")
        self.get_logger().info("   UR5e Unity Bridge Diagnostics Mode     ")
        self.get_logger().info("==========================================")
        self.get_logger().info("Listening on: /unity/target_pose")
        self.get_logger().info("Publishing to: /unity/health")

    def timer_callback(self):
        msg = String()
        msg.data = f"ROS2 Heartbeat - {self.get_clock().now().to_msg().sec}"
        self.health_pub.publish(msg)
        
        self.heartbeat_count += 1
        if self.heartbeat_count % 10 == 0:
            self.get_logger().info(f"Alive: Sending Heartbeat to Unity (count: {self.heartbeat_count})")

    def joint_state_callback(self, msg: JointState):
        """
        ROS의 /joint_states를 받아서 Unity가 이해하기 쉽게 재가공하여 보냅니다.
        특히 effort에 포함된 NaN 값을 0.0으로 바꿉니다.
        """
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = msg.name
        new_msg.position = msg.position
        new_msg.velocity = msg.velocity
        new_msg.effort = [0.0] * len(msg.name)
        
        self.unity_joint_pub.publish(new_msg)

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Target Received: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
        self.send_move_goal(msg)

    def joint_command_callback(self, msg: JointState):
        pass # 현재는 타입 추론용으로만 사용 (Unity -> ROS)

    def send_move_goal(self, target_pose: PoseStamped):
        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("MoveIt2 action server ('move_action') not found!")
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator" # ur_moveit_config 기준 표준 이름
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Goal Constraints
        pos_con = PositionConstraint()
        pos_con.header = target_pose.header
        pos_con.link_name = "tool0"
        pos_con.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.05])) # 5cm 오차 허용
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        pos_con.weight = 1.0
        
        ori_con = OrientationConstraint()
        ori_con.header = target_pose.header
        ori_con.link_name = "tool0"
        ori_con.orientation = target_pose.pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.1 # 약 5.7도 허용
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)
        
        # 추가적인 안정성 설정
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.5 # 속도 50%
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        self._send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by MoveIt2')
            return
        self.get_logger().info('Goal ACCEPTED. Starting execution...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS: Robot moved to target.')
        else:
            self.get_logger().error(f'FAILURE: MoveIt error code {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    node = UR5eUnityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
