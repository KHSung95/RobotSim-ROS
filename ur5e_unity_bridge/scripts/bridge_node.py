#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Int8
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.srv import GetCartesianPath # Removed service import

class UR5eUnityBridge(Node):
    def __init__(self):
        super().__init__('ur5e_unity_bridge')
        
        # QoS 프로파일 정의 (MoveIt Controller와의 호환성 고려)
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Publishers (ROS -> Unity / Robot)
        self.unity_health_pub = self.create_publisher(String, '/unity/health', 10)
        self.unity_joint_state_pub = self.create_publisher(JointState, '/unity/joint_states', 10)
        
        # Robot Control Publishers
        self.robot_traj_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.robot_servo_twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', self.qos)
        self.robot_servo_jog_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', self.qos)
        
        # 2. Subscribers (Unity -> ROS)
        self.unity_target_sub = self.create_subscription(PoseStamped, '/unity/target_pose', self.unity_target_pose_callback, 10)
        self.unity_joint_traj_sub = self.create_subscription(JointState, '/joint_commands', self.unity_joint_traj_callback, 10)
        self.unity_joint_jog_sub = self.create_subscription(JointJog, '/unity/joint_jog', self.unity_joint_jog_callback, self.qos)
        self.unity_twist_sub = self.create_subscription(TwistStamped, '/servo_node/delta_twist_cmds_unity', self.unity_twist_callback, self.qos)
        self.unity_trajectory_sub = self.create_subscription(JointTrajectory, '/unity/joint_trajectory', self.unity_trajectory_callback, 10)
        
        # 3. Service Bridge (Removed Cartesian Proxy to prevent conflict)
        # Unity should use /move_robot_to_pose served by moveit_bridge.py
        
        # 4. Status Subscribers (Robot -> Bridge)
        self.robot_joint_state_sub = self.create_subscription(JointState, '/joint_states', self.robot_joint_state_callback, 10)
        self.robot_servo_status_sub = self.create_subscription(Int8, '/servo_node/status', self.robot_servo_status_callback, 10)
        
        # Status & Monitoring Variables
        self.msg_count = {"twist": 0, "joint_cmd": 0, "joint_state": 0, "jog": 0}
        self.last_servo_cmd_time = self.get_clock().now()
        self.current_servo_status = 0 # 0: Normal

        # 5. Service Clients (For Servo Control)
        self.start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        
        # 6. Automatic Startup Logic
        self.servo_started = False
        self.auto_start_timer = self.create_timer(2.0, self.auto_start_callback)
        
        # 7. Throttling Logic (To prevent rosbridge overload)
        self.last_joint_pub_time = 0.0
        self.pub_interval = 1.0 / 30.0 # 30Hz limit for Unity

        self.get_logger().info("==========================================")
        self.get_logger().info("   UR5e Unity Bridge [v2.2 - Optimized]   ")
        self.get_logger().info("==========================================")


    def robot_joint_state_callback(self, msg: JointState):
        """로봇 -> Unity 상태 동기화 (30Hz Throttling 적용)"""
        current_time = time.time()
        if (current_time - self.last_joint_pub_time) < self.pub_interval:
            return # 전송 빈도 제한
            
        self.last_joint_pub_time = current_time
        self.msg_count["joint_state"] += 1
        new_msg = JointState()
        # [중요] Unity 타임스탬프 문제를 해결하기 위해 ROS 시간으로 덮어씀
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "base_link"
        new_msg.name = msg.name
        new_msg.position = msg.position
        new_msg.velocity = msg.velocity
        new_msg.effort = [0.0] * len(msg.name)
        self.unity_joint_state_pub.publish(new_msg)
        
        # 주기적으로 통신 상태 확인을 위한 디버그 로그 (100번마다 한 번)
        if self.msg_count["joint_state"] % 100 == 0:
            self.get_logger().info(f"Published 100 JointStates to Unity. Total: {self.msg_count['joint_state']}", once=False)

    def unity_twist_callback(self, msg: TwistStamped):
        """Unity -> Servo Twist 명령 (Cartesian Jog)"""
        self.msg_count["twist"] += 1
        
        # [수비적 설계] 에러 상태(특이점, 충돌 등)일 때는 Twist 명령을 무시함
        # 불필요한 연산과 경고 로그를 줄이고, JointJog를 통한 탈출을 유도함
        if self.current_servo_status != 0:
            return 

        self.last_servo_cmd_time = self.get_clock().now()
        
        # 타임스탬프 갱신 (Servo 타임아웃 방지)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.robot_servo_twist_pub.publish(msg)

        # [수정됨] 과잉 호출 방지: try_start_servo() 제거됨

    def unity_joint_jog_callback(self, msg: JointJog):
        """Unity -> Servo Joint Jog 명령 (관절 개별 제어)"""
        self.msg_count["jog"] += 1
        self.last_servo_cmd_time = self.get_clock().now()
        
        # 타임스탬프 갱신
        msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id: msg.header.frame_id = "base_link"
        
        # [참고] JointJog는 특이점 탈출용이므로 status != 0 이어도 퍼블리시 함
        self.robot_servo_jog_pub.publish(msg)

        # [수정됨] 과잉 호출 방지: try_start_servo() 제거됨

    def unity_joint_traj_callback(self, msg: JointState):
        """Unity -> Scaled Controller (Move to Pose 등)"""
        # Debug: Check if callback is triggered
        if not msg.name or not msg.position:
            # self.get_logger().info(f"Empty data from Unity: name={len(msg.name)}, pos={len(msg.position)}")
            return
            
        # 상세 수치 로깅 추가
        joint_info = [f"{n}: {p:.4f}" for n, p in zip(msg.name, msg.position)]
        self.get_logger().info(f"Joint command received:\n" + "\n".join(joint_info))
        
        self.msg_count["joint_cmd"] += 1
        
        # Servo와 Controller 간의 충돌 방지 (Servo 사용 직후엔 잠시 대기)
        dt = (self.get_clock().now() - self.last_servo_cmd_time).nanoseconds / 1e9
        if dt < 0.5:
            self.get_logger().warn("Skipping joint command: Too soon after Servo command")
            return

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = msg.name 
        point = JointTrajectoryPoint()
        point.positions = msg.position
        # 안전을 위해 빈 속도 배열이라도 채워서 보냄
        point.velocities = [0.0] * len(msg.position)
        point.time_from_start.nanosec = 200000000 # 0.2s
        traj_msg.points = [point]
        self.robot_traj_pub.publish(traj_msg)

    def unity_trajectory_callback(self, msg: JointTrajectory):
        """Unity에서 계산된 궤적(Cartesian Path 등)을 바로 실행"""
        self.get_logger().info("Unity Trajectory 실행 요청 수신")
        
        dt = (self.get_clock().now() - self.last_servo_cmd_time).nanoseconds / 1e9
        if dt < 0.5: return

        msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id: msg.header.frame_id = "base_link"
        self.robot_traj_pub.publish(msg)

    # async def compute_cartesian_path_callback(self, request, response):
    #     """REMOVED: Use moveit_bridge.py's /move_robot_to_pose instead"""
    #     pass

    def robot_servo_status_callback(self, msg: Int8):
        """Servo 상태 모니터링 및 로깅"""
        # 상태가 변하지 않았으면 리턴 (로그 스팸 방지)
        if msg.data == self.current_servo_status:
            return
            
        self.current_servo_status = msg.data
        status_map = {0: "정상(Normal)", 1: "경고(Warn)", 2: "특이점 정지(Singularity Halt)", 3: "경계", 4: "한계 정지(Limit Halt)", 5: "충돌 위험(Collision)", 6: "오류"}
        status_str = status_map.get(msg.data, f"알 수 없음 ({msg.data})")
        
        # Unity 화면에 표시할 수 있도록 Health 토픽으로도 전송 추천
        health_msg = String()
        health_msg.data = f"Servo Status: {status_str}"
        self.unity_health_pub.publish(health_msg)

        if msg.data == 0:
            self.get_logger().info(f"Servo 상태 회복: {status_str}")
        else:
            self.get_logger().warn(f"Servo 상태 변경: {status_str} -> 조치 필요")

    def auto_start_callback(self):
        """노드 시작 시 Servo를 자동으로 켜주는 로직 (1회성)"""
        if self.servo_started:
            self.auto_start_timer.cancel()
            return
            
        if self.start_servo_client.service_is_ready():
            self.get_logger().info("Servo 서비스 감지됨. 자동 시작 시도...")
            self.try_start_servo()
            self.servo_started = True
            self.auto_start_timer.cancel()
        else:
            self.get_logger().info("Servo 서비스 대기 중...", once=True)

    def try_start_servo(self):
        """Servo 활성화 요청"""
        if not self.start_servo_client.service_is_ready():
            return
        self.get_logger().info("Servo Enable 요청 전송 (/servo_node/start_servo)")
        req = Trigger.Request()
        self.start_servo_client.call_async(req)

    def unity_target_pose_callback(self, msg: PoseStamped):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = UR5eUnityBridge()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error in bridge node: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()