#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from franka_control_package.action import MoveRobot
from franka_control_package.msg import MoveRobotInterface
from franka_msgs.action import Grasp, Homing, Move
from enum import Enum
import sys
import threading

class GripperAction(Enum):
    NONE = 0
    GRASP = 1
    HOMING = 2
    MOVE = 3

class ArmActionType(Enum):
    PLAN_ONLY = 0
    EXECUTE_ONLY = 1
    PLAN_AND_EXECUTE = 2

class TaskState:
    """개별 태스크의 상태를 관리하는 클래스"""
    def __init__(self, task_id, msg):
        self.task_id = task_id
        self.msg = msg
        self.arm_completed = False
        self.gripper_completed = False
        self.arm_success = False
        self.gripper_success = False
        self.arm_result_future = None
        self.gripper_future = None

class MoveRobotInterfaceNode(Node):
    def __init__(self):
        super().__init__('move_robot_interface_node')
        
        # Declare parameters
        self.declare_parameter("franka_gripper_node_name", "/franka_gripper")
        self.declare_parameter("use_gripper", True)

        # Task management
        self._task_counter = 0
        self._active_tasks = {}  # task_id -> TaskState
        self._task_lock = threading.Lock()

        # Create subscriber for robot movement topic
        self._move_robot_subscriber = self.create_subscription(
            MoveRobotInterface,
            '~/move_robot_interface',
            self.move_robot_interface_callback,
            10
        )

        # Create action client for robot arm
        self._robot_arm_action_client = ActionClient(self, MoveRobot, 'move_robot_action')
        
        self.get_logger().info('Waiting for action server (MoveRobot)...')
        
        if not self._robot_arm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server(MoveRobot) not available after 10 seconds!')
            return
        
        self.get_logger().info('Action server(MoveRobot) found!')

        # Create action client for robot gripper
        franka_gripper_node_name = self.get_parameter("franka_gripper_node_name").get_parameter_value().string_value
        self._use_gripper = self.get_parameter("use_gripper").get_parameter_value().bool_value
        if self._use_gripper:
            self.get_logger().info('Initializing Gripper action clients')
            self._grasp_action_client = ActionClient(self, Grasp, f'{franka_gripper_node_name}/grasp')
            self._homing_action_client = ActionClient(self, Homing, f'{franka_gripper_node_name}/homing')
            self._moving_action_client = ActionClient(self, Move, f'{franka_gripper_node_name}/move')

            # connect to servers
            action_clients = [
                (self._grasp_action_client, 'Grasp'),
                (self._homing_action_client, 'Homing'), 
                (self._moving_action_client, 'Move')
            ]

            for client, name in action_clients:
                self.get_logger().info(f'Waiting for action server ({name})...')
                if not client.wait_for_server(timeout_sec=10.0):
                    self.get_logger().error(f'Action server({name}) not available after 10 seconds!')
                    return
                self.get_logger().info(f'Action server ({name}) found!')
            
        else:
            self.get_logger().info('Not using a gripper')

        self.get_logger().info('Move Robot Interface Node initialized successfully.')

    def move_robot_interface_callback(self, msg):
        """비동기 방식으로 변경된 콜백 함수"""
        with self._task_lock:
            task_id = self._task_counter
            self._task_counter += 1
            task_state = TaskState(task_id, msg)
            self._active_tasks[task_id] = task_state

        self.get_logger().info(f'Starting task {task_id}')
        
        target_pose = msg.target_pose
        gripper_action_type = GripperAction(msg.gripper_action_type)
        
        # Send the arm goal asynchronously
        self.send_robot_arm_goal_async(
            task_id=task_id,
            x=target_pose.position.x,
            y=target_pose.position.y,
            z=target_pose.position.z,
            ori_x=target_pose.orientation.x,
            ori_y=target_pose.orientation.y,
            ori_z=target_pose.orientation.z,
            ori_w=target_pose.orientation.w,
            action_type=ArmActionType.PLAN_AND_EXECUTE.value,
            gripper_action=gripper_action_type
        )

    def send_robot_arm_goal_async(self, task_id, x=0.3, y=0.0, z=0.5, 
                                 ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0,
                                 action_type=0, gripper_action=GripperAction.NONE):
        """비동기 방식으로 로봇 팔 목표 전송"""
        
        # Create goal message
        goal_msg = MoveRobot.Goal()
        
        target_x, target_y, target_z, target_ori_x, target_ori_y, target_ori_z, target_ori_w = \
            self.transform_end_effector_pose(x, y, z, ori_x, ori_y, ori_z, ori_w)

        # Set target pose
        goal_msg.target_pose.position.x = target_x
        goal_msg.target_pose.position.y = target_y
        goal_msg.target_pose.position.z = target_z
        goal_msg.target_pose.orientation.w = target_ori_w
        goal_msg.target_pose.orientation.x = target_ori_x
        goal_msg.target_pose.orientation.y = target_ori_y
        goal_msg.target_pose.orientation.z = target_ori_z
        goal_msg.action_type = action_type
        
        action_names = {
            MoveRobot.Goal.PLAN_ONLY: "PLAN_ONLY",
            MoveRobot.Goal.EXECUTE_ONLY: "EXECUTE_ONLY", 
            MoveRobot.Goal.PLAN_AND_EXECUTE: "PLAN_AND_EXECUTE"
        }
        
        self.get_logger().info(
            f'Task {task_id}: Sending goal - Position: [{x:.3f}, {y:.3f}, {z:.3f}] '
            f'Orientation: [{ori_w:.3f}, {ori_x:.3f}, {ori_y:.3f}, {ori_z:.3f}] '
            f'with action: {action_names.get(action_type, "UNKNOWN")}'
        )
        
        # Send goal with task-specific callback
        send_goal_future = self._robot_arm_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=lambda feedback_msg, task_id=task_id: self.arm_feedback_callback(feedback_msg, task_id)
        )
        send_goal_future.add_done_callback(
            lambda future, task_id=task_id, gripper_action=gripper_action: 
            self.arm_goal_response_callback(future, task_id, gripper_action)
        )
        
        return send_goal_future

    def arm_goal_response_callback(self, future, task_id, gripper_action):
        """태스크별 팔 목표 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Task {task_id}: Robot arm goal rejected')
            self.cleanup_task(task_id)
            return
        
        self.get_logger().info(f'Task {task_id}: Robot arm goal accepted')
        
        # Get result with task-specific callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, task_id=task_id, gripper_action=gripper_action: 
            self.arm_get_result_callback(future, task_id, gripper_action)
        )
        
        # Store result future in task state
        with self._task_lock:
            if task_id in self._active_tasks:
                self._active_tasks[task_id].arm_result_future = result_future

    def arm_get_result_callback(self, future, task_id, gripper_action):
        """태스크별 팔 결과 콜백"""
        result = future.result().result
        
        with self._task_lock:
            if task_id not in self._active_tasks:
                return
            
            task_state = self._active_tasks[task_id]
            task_state.arm_completed = True
            task_state.arm_success = result.success

        if result.success:
            self.get_logger().info(f'Task {task_id}: Robot arm goal succeeded')
            
            # Start gripper action if needed
            if self._use_gripper and gripper_action != GripperAction.NONE:
                self.get_logger().info(f'Task {task_id}: Starting gripper action')
                gripper_future = self.send_gripper_goal_based_on_action(task_id, gripper_action, task_state.msg)
                
                with self._task_lock:
                    if task_id in self._active_tasks:
                        self._active_tasks[task_id].gripper_future = gripper_future
            else:
                self.get_logger().info(f'Task {task_id}: No gripper action needed, task completed')
                self.cleanup_task(task_id)
        else:
            self.get_logger().error(f'Task {task_id}: Robot arm goal failed: {result.error}')
            self.cleanup_task(task_id)

    def send_gripper_goal_based_on_action(self, task_id, gripper_action, msg):
        """gripper action에 따라 적절한 goal 전송 (태스크 ID 포함)"""
        if gripper_action == GripperAction.GRASP:
            return self.send_grasp_goal(
                task_id=task_id,
                width=msg.gripper_width if hasattr(msg, 'gripper_width') else 0.0,
                speed=msg.gripper_speed if hasattr(msg, 'gripper_speed') else 0.03,
                force=msg.gripper_force if hasattr(msg, 'gripper_force') else 50.0
            )
        elif gripper_action == GripperAction.HOMING:
            return self.send_homing_goal(task_id)
        elif gripper_action == GripperAction.MOVE:
            return self.send_move_goal(
                task_id=task_id,
                width=msg.gripper_width if hasattr(msg, 'gripper_width') else 0.08,
                speed=msg.gripper_speed if hasattr(msg, 'gripper_speed') else 0.1
            )
        return None

    def send_grasp_goal(self, task_id, width=0.00, speed=0.03, force=50.0, epsilon_inner=0.005, epsilon_outer=0.005):
        """Send a grasp goal to the gripper"""
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon_inner
        goal_msg.epsilon.outer = epsilon_outer
        
        self.get_logger().info(f'Task {task_id}: Sending grasp goal - Width: {width}, Speed: {speed}, Force: {force}')
        
        grasp_goal_future = self._grasp_action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, task_id=task_id: self.grasp_feedback_callback(feedback_msg, task_id)
        )
        grasp_goal_future.add_done_callback(
            lambda future, task_id=task_id: self.grasp_goal_response_callback(future, task_id)
        )
        
        return grasp_goal_future

    def send_homing_goal(self, task_id):
        """Send a homing goal to the gripper"""
        goal_msg = Homing.Goal()
        
        self.get_logger().info(f'Task {task_id}: Sending homing goal to gripper')
        
        homing_goal_future = self._homing_action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, task_id=task_id: self.homing_feedback_callback(feedback_msg, task_id)
        )
        homing_goal_future.add_done_callback(
            lambda future, task_id=task_id: self.homing_goal_response_callback(future, task_id)
        )
        
        return homing_goal_future

    def send_move_goal(self, task_id, width=0.08, speed=0.1):
        """Send a move goal to the gripper"""
        goal_msg = Move.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        
        self.get_logger().info(f'Task {task_id}: Sending move goal - Width: {width}, Speed: {speed}')
        
        move_goal_future = self._moving_action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, task_id=task_id: self.move_feedback_callback(feedback_msg, task_id)
        )
        move_goal_future.add_done_callback(
            lambda future, task_id=task_id: self.move_goal_response_callback(future, task_id)
        )
        
        return move_goal_future

    def cleanup_task(self, task_id):
        """태스크 정리"""
        with self._task_lock:
            if task_id in self._active_tasks:
                del self._active_tasks[task_id]
        self.get_logger().info(f'Task {task_id}: Completed and cleaned up')

    # Robot arm callbacks (updated with task_id)
    def arm_feedback_callback(self, feedback_msg, task_id):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Task {task_id}: Robot arm feedback: {feedback.current_status}')

    # Gripper callbacks (updated with task_id)
    def grasp_goal_response_callback(self, future, task_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Task {task_id}: Grasp goal rejected')
            self.cleanup_task(task_id)
            return
        
        self.get_logger().info(f'Task {task_id}: Grasp goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, task_id=task_id: self.grasp_get_result_callback(future, task_id)
        )

    def grasp_get_result_callback(self, future, task_id):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Task {task_id}: Grasp succeeded')
        else:
            self.get_logger().error(f'Task {task_id}: Grasp failed: {result.error}')
        self.cleanup_task(task_id)

    def grasp_feedback_callback(self, feedback_msg, task_id):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Task {task_id}: Grasp feedback - current_width: {feedback.current_width}')

    def homing_goal_response_callback(self, future, task_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Task {task_id}: Homing goal rejected')
            self.cleanup_task(task_id)
            return
        
        self.get_logger().info(f'Task {task_id}: Homing goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, task_id=task_id: self.homing_get_result_callback(future, task_id)
        )

    def homing_get_result_callback(self, future, task_id):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Task {task_id}: Homing succeeded')
        else:
            self.get_logger().error(f'Task {task_id}: Homing failed: {result.error}')
        self.cleanup_task(task_id)

    def homing_feedback_callback(self, feedback_msg, task_id):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Task {task_id}: Homing feedback - current_width: {feedback.current_width}')

    def move_goal_response_callback(self, future, task_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Task {task_id}: Move goal rejected')
            self.cleanup_task(task_id)
            return
        
        self.get_logger().info(f'Task {task_id}: Move goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, task_id=task_id: self.move_get_result_callback(future, task_id)
        )

    def move_get_result_callback(self, future, task_id):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Task {task_id}: Move succeeded')
        else:
            self.get_logger().error(f'Task {task_id}: Move failed: {result.error}')
        self.cleanup_task(task_id)

    def move_feedback_callback(self, feedback_msg, task_id):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Task {task_id}: Move feedback - current_width: {feedback.current_width}')

    # Utility function (unchanged)
    def transform_end_effector_pose(self, x, y, z, ori_x, ori_y, ori_z, ori_w):
        """
        Transform end-effector pose from fr3_hand frame to fr3_link8 frame.
        
        Applies a 45-degree Z-axis rotation to compensate for the frame offset
        between the target planning frame (fr3_link8) and the actual end-effector frame (fr3_hand).
        
        Args:
            x, y, z: Target position coordinates in fr3_hand frame
            ori_x, ori_y, ori_z, ori_w: Target orientation quaternion in fr3_hand frame
        
        Returns:
            tuple: Transformed (x, y, z, ori_x, ori_y, ori_z, ori_w) for fr3_link8 frame
        """
        from scipy.spatial.transform import Rotation as R
        import numpy as np
        
        # 입력 orientation을 Rotation 객체로 변환
        input_rotation = R.from_quat([ori_x, ori_y, ori_z, ori_w])
        
        # Z축 45도 회전 변환
        transform_rotation = R.from_euler('z', 45, degrees=True)
        
        # 회전 합성: transformed = input * transform
        transformed_rotation = input_rotation * transform_rotation
        
        # 결과를 quaternion으로 변환
        transformed_quat = transformed_rotation.as_quat()
        
        # 위치는 그대로 유지
        return x, y, z, transformed_quat[0], transformed_quat[1], transformed_quat[2], transformed_quat[3]

def print_usage():
    print("\nUsage:")
    print("  python3 interactive_move_client.py <x> <y> <z> <roll> <pitch> <yaw> <gripper_action> <arm_action_type> [gripper_params...]")
    print("\nParameters:")
    print("  x, y, z: Target position coordinates (float)")
    print("  roll, pitch, yaw: Target orientation in degrees (float)")
    print("  gripper_action:")
    for action in GripperAction:
        print(f"    {action.value}={action.name}")
    print("  arm_action_type:")
    for action in ArmActionType:
        print(f"    {action.value}={action.name}")
    print("  gripper_params (optional):")
    print("    For grasp: width speed force")
    print("    For move: width speed")
    print("\nExamples:")
    print(f"  python3 interactive_move_client.py 0.3 0.2 0.5 0.0 0.0 45.0 {GripperAction.GRASP.value} {ArmActionType.PLAN_AND_EXECUTE.value} 0.08 0.1 60.0  # Move arm and grasp")
    print(f"  python3 interactive_move_client.py 0.28 -0.2 0.5 30.0 -15.0 0.0 {GripperAction.HOMING.value} {ArmActionType.PLAN_ONLY.value}  # Plan arm movement and home gripper")
    print(f"  python3 interactive_move_client.py 0.3 0.0 0.5 0.0 0.0 90.0 {GripperAction.MOVE.value} {ArmActionType.PLAN_AND_EXECUTE.value} 0.04 0.1  # Move arm and set gripper width")
    print(f"  python3 interactive_move_client.py 0.3 0.0 0.5 0.0 0.0 0.0 {GripperAction.NONE.value} {ArmActionType.PLAN_AND_EXECUTE.value}  # Only move arm")
    print("")

def euler_to_quaternion(roll, pitch, yaw, degrees=True):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.
    
    Args:
        roll: Rotation around x-axis
        pitch: Rotation around y-axis  
        yaw: Rotation around z-axis
        degrees: If True, angles are in degrees. If False, in radians.
    
    Returns:
        tuple: (qx, qy, qz, qw) quaternion components
    """
    from scipy.spatial.transform import Rotation as R
    
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=degrees)
    quat = rotation.as_quat()
    
    return quat[0], quat[1], quat[2], quat[3]  # qx, qy, qz, qw

def main(args=None):    
    rclpy.init(args=args)
    
    move_robot_interface_node = MoveRobotInterfaceNode()
    
    rclpy.spin(move_robot_interface_node)
    
    move_robot_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()