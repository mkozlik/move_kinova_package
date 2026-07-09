import rclpy
from rclpy.node import Node
import time
import yaml
from pathlib import Path

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Twist
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_msgs.msg import Float32, String, Int32

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from move_kinova_msgs.action import SimpleMove  
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand

from moveit_msgs.msg import PlanningScene, CollisionObject, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from hero_custom_msgs.srv import PickObject, ReleaseObject, SetThreatLevel

from moveit_msgs.action import MoveGroup

import tf2_ros
import tf2_geometry_msgs


sensor_reading = 4.2  # Global variable to store the latest sensor reading

class States:
    IDLE = 0
    APPROACHING = 1
    GRABBING = 2
    PLACING = 3
    POUCHING = 4
    HOMING = 5
    NAVIGATING = 6
    SPRAYING = 7
    SNIFFING = 8


class SensorReaderNode(Node):
    def __init__(self):
        super().__init__('sensor_reader_node')
        self.subscription = self.create_subscription(
            Float32,
            '/poly5_topic',
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg: Float32):
        global sensor_reading
        sensor_reading = msg.data
        #self.get_logger().info(f"Received sensor reading: {sensor_reading}")


class KinovaStateMachineNode(Node):
    def __init__(self):
        super().__init__('kinova_state_machine_node')

        # ---- Subscriber ----
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/container/all/bounding_box_3d',
            self.detection_callback,
            10
        )

        self.object_point_subscription = self.create_subscription(
            PointStamped,
            '/container/centre_point_3d',
            self.object_point_callback,
            10
        )

        self.object_subscription = self.create_subscription(
            String,
            '/current_object',
            self.object_callback,
            10
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.state_machine_callback)

        # ---- Service Clients ---- 
        self.pouch_service = self.create_service(
            SetBool,
            '/trigger_pouch',
            self.pouch_service_callback
        )

        self.sniff_service = self.create_service(
            SetBool,
            '/trigger_sniff',
            self.sniff_service_callback
        )


        self.grab_service = self.create_service(
            SetBool,
            '/trigger_grab',
            self.grab_service_callback
        )

        self.place_back_service = self.create_service(
            SetBool,
            '/trigger_place_back',
            self.place_back_service_callback
        )

        self.spray_service = self.create_service(
            SetBool,
            '/trigger_spray',
            self.spray_service_callback
        )

        self.pick_object_client = self.create_client(SetBool, '/unity_pick_object')
        self.pouch_object_client = self.create_client(SetBool, '/unity_pouch_object')
        self.place_back_client = self.create_client(SetBool, '/unity_place_back_object')
        self.set_threat_client = self.create_client(SetThreatLevel, '/unity_set_threat_level')
        self.spray_client = self.create_client(SetBool, '/unity_spray_object')
        self.pick_spray_client = self.create_client(SetBool, '/unity_pick_spray')
        self.place_spray_client = self.create_client(SetBool, '/unity_place_spray')
        self.robot_joint_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_angle_publisher = self.create_publisher(Int32, '/servo_angle', 10)
        # 10 Hz tick: 170 while spraying (at first pose + iterating spray
        # waypoints, SPRAYING substeps 6-7), 90 otherwise.
        self.servo_timer = self.create_timer(0.1, self._publish_servo_angle)

        # ---- Action Client ----
        self.action_client = ActionClient(
            self,
            SimpleMove,
            'grab_object'
        )

        self.planning_scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        self.gripper_client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')

        # ---- TF Buffer ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for latest pose
        self.latest_pose = None
        self.latest_centre_point = None
        self.latest_bbox_size = None
        self.transformed_pose = None
        self.picked_pose = None
        self.latest_detection = None
        self.picking_object = True
        self.state = States.HOMING
        self.state_transitioned = True
        self.pouch_pressed = False
        self.grab_pressed = False
        self.sniff_pressed = False
        self.place_back_pressed = False
        self.clock = 0.0
        self.grab_done = False
        self.grab_succeeded = False
        self.spray_pressed = False
        self.spray_substep = 0
        self.spray_pose_idx = 0

        # ---- Base-alignment tuning (open loop) ----
        # /centre_point_3d is in camera_color_optical_frame (+X right, +Z fwd).
        # We snapshot the point once when SPRAYING starts, compute the required
        # planar displacement, and drive the base at a fixed speed for the
        # computed duration. Live /centre_point_3d updates are ignored so
        # stale-point readings don't abort the microadjustment.
        self.base_align_eps_x = 0.010     # m (skip strafe if disp below this)
        self.base_align_eps_z = 0.010     # m (skip forward if disp below this)
        self.base_openloop_speed = 0.08   # m/s (fixed drive speed per axis)
        self.base_align_timeout = 25.0    # s
        self.base_align_capture_timeout = 3.0  # s to wait for first /centre_point_3d

        # ---- Open-loop base motion state ----
        self._openloop_captured = False
        self._openloop_v_x = 0.0
        self._openloop_v_y = 0.0
        self._openloop_t_x = 0.0
        self._openloop_t_y = 0.0
        self._openloop_start_clock = 0.0

        # ---- Spray-sequence timing ----
        self.spray_pose_duration = 3.0    # JointTrajectory duration between spray poses
        self.spray_pose_settle = 3.5      # wait after publishing spray pose
        self.spray_start_delay = 2.0      # wait at first pose after servo -> 170
        # Hold at the last spray pose after the servo releases (servo -> 90).
        # The physical spray keeps flowing for a moment after the trigger is
        # released, so we stay put (still aimed at the target) long enough for
        # it to fully stop before the arm carries the bottle away to place-back.
        self.spray_stop_delay = 4.0       # wait at last pose after servo -> 90

        # read poses from yaml
        yaml_path = Path(__file__).parent / 'config' / 'robot_poses.yaml' # in config folder
        with open(yaml_path, 'r') as file:
            self.predefined_poses = yaml.safe_load(file)
            print(f"Loaded predefined poses: {self.predefined_poses.get('home', 'Not found').get('joint', 'Not found')[:3]}")

        # Spray waypoint sequence: ordered list of joint targets from
        # robot_spray_poses.yaml (pose_1, pose_2, ...).
        spray_yaml = Path(__file__).parent / 'config' / 'robot_spray_poses.yaml'
        with open(spray_yaml, 'r') as f:
            spray_data = yaml.safe_load(f) or {}
        def _pose_key(name: str) -> int:
            try:
                return int(name.rsplit('_', 1)[1])
            except (ValueError, IndexError):
                return 0
        self.spray_poses = [
            spray_data[name]['joint']
            for name in sorted(spray_data.keys(), key=_pose_key)
        ]
        print(f"Loaded {len(self.spray_poses)} spray poses")

        # Desired object position (in camera_color_optical_frame at home pose)
        # for the base-alignment step.
        base_yaml = Path(__file__).parent / 'config' / 'base_position.yaml'
        with open(base_yaml, 'r') as f:
            base_data = yaml.safe_load(f) or {}
        pre = base_data.get('pre_decontamination_pose', {})
        self.base_target_x = float(pre.get('x', 0.0))
        self.base_target_z = float(pre.get('z', 0.5))
        print(f"Base target: x={self.base_target_x:.3f}, z={self.base_target_z:.3f} "
              f"(camera_color_optical_frame)")

        home_joints = self.predefined_poses['home']['joint']
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(home_joints))]
        point = JointTrajectoryPoint()
        point.positions = home_joints
        point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
        traj_msg.points.append(point)
        self.robot_joint_publisher.publish(traj_msg)


    def joint_state_callback(self, msg: JointState):
        # Save latest joint states if needed for feedback or state estimation
        self.latest_joint_states = msg
        pass


    def object_point_callback(self, msg: PointStamped):
        # Dedicated store for the raw camera-frame centre point used by the
        # base-alignment step. Keep it separate from latest_pose, which
        # detection_callback overwrites with a PoseStamped.
        self.latest_centre_point = msg
        self.latest_pose = msg
        try:
            # Transform the point to the robot's base frame but point itself cannot be tranformed, so we create a PoseStamped with the point as the position and a default orientation
            pose_msg = PoseStamped()
            pose_msg.header = msg.header

            pose_msg.pose.position.x = msg.point.x
            pose_msg.pose.position.y = msg.point.y
            pose_msg.pose.position.z = msg.point.z

            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            transform = self.tf_buffer.lookup_transform(
                'base_link',
                pose_msg.header.frame_id,
                rclpy.time.Time()
            )

            self.transformed_pose = self.tf_buffer.transform(
                pose_msg,
                'world',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"TF transformation failed: {str(e)}")
            self.transformed_pose = None


    def pouch_service_callback(self, request, response):
        if self.state != States.HOMING:
            response.success = False
            response.message = "Cannot pouch: Robot not in HOMING state"
            return response
        try:
            # For demonstration, let's just log the release action
            self.get_logger().info("Pouch service called")
            self.pouch_pressed = True  # Set the flag to trigger pouching in the state machine
        except Exception as e:
            self.get_logger().error(f"Pouch failed: {str(e)}")
            response.success = False
            response.message = "Error in pouch"
            return response

        response.success = True
        response.message = "Pouch triggered"
        return response

    def sniff_service_callback(self, request, response):
        if self.state != States.HOMING:
            response.success = False
            response.message = "Cannot sniff: Robot not in HOMING state"
            return response
        try:
            # For demonstration, let's just log the sniff action
            self.get_logger().info("Sniff service called")
            self.sniff_pressed = True  # Set the flag to trigger sniffing in the state machine
        except Exception as e:
            self.get_logger().error(f"Sniff failed: {str(e)}")
            response.success = False
            response.message = "Error in sniff"
            return response

        response.success = True
        response.message = "Sniff triggered"
        return response

    def grab_service_callback(self, request, response):
        if self.state != States.IDLE:
            response.success = False
            response.message = "Cannot grab: Robot not in IDLE state"
            return response
        try:
            self.get_logger().info("Grab service called")
            self.grab_pressed = True  # Set the flag to trigger grabbing in the state machine
        except Exception as e:
            self.get_logger().error(f"Grab failed: {str(e)}")
            response.success = False
            response.message = "Error in grab"
            return response

        response.success = True
        response.message = "Grab triggered"
        return response

    def place_back_service_callback(self, request, response):
        if self.state != States.HOMING:
            response.success = False
            response.message = "Cannot place back: Robot not in HOMING state"
            return response
        try:
            self.get_logger().info("Place back service called")
            self.place_back_pressed = True  # Set the flag to trigger placing back in the state machine
        except Exception as e:
            self.get_logger().error(f"Place back failed: {str(e)}")
            response.success = False
            response.message = "Error in place back"
            return response

        response.success = True
        response.message = "Place back triggered"
        return response

    def spray_service_callback(self, request, response):
        if self.state != States.IDLE:
            response.success = False
            response.message = "Cannot spray: Robot not in IDLE state"
            return response
        try:
            self.get_logger().info("Spray service called")
            self.spray_pressed = True  # Set the flag to trigger spraying in the state machine
        except Exception as e:
            self.get_logger().error(f"Spray failed: {str(e)}")
            response.success = False
            response.message = "Error in spray"
            return response

        response.success = True
        response.message = "Spray triggered"
        return response


    def gripper_command(self, command):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = command  # 0 for open, 1 for close
        self.gripper_client.send_goal_async(goal_msg)

    def _publish_joint_traj(self, joints, duration: float = 3.0) -> None:
        """Publish a single-point JointTrajectory for the arm joints."""
        traj = JointTrajectory()
        traj.joint_names = [f'joint_{i+1}' for i in range(len(joints))]
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        traj.points.append(point)
        self.robot_joint_publisher.publish(traj)

    def _publish_servo_angle(self) -> None:
        """Continuously publish the spray-trigger servo angle.

        Trigger pressed during the spray-pose sequence: SPRAYING substeps
        6 (arrived at first spray pose + start delay) and 7 (iterating
        through the remaining spray poses). Released otherwise.
        """
        msg = Int32()
        spraying_now = (self.state == States.SPRAYING and
                        self.spray_substep in (6, 7))
        msg.data = 160 if spraying_now else 90
        self.servo_angle_publisher.publish(msg)

    def _stop_base(self) -> None:
        """Publish a zero Twist so the base holds position."""
        self.cmd_vel_publisher.publish(Twist())

    def _notify_unity_done(self, client, label: str) -> None:
        """Fire-and-forget SetBool(True) telling Unity/the operator an action finished."""
        request = SetBool.Request()
        request.data = True
        client.call_async(request)
        self.get_logger().info(f"Notified operator: {label} finished")

    @staticmethod
    def _clamp(v: float, limit: float) -> float:
        if v > limit:
            return limit
        if v < -limit:
            return -limit
        return v

    def _base_align_step(self) -> bool:
        """Open-loop planar microadjustment of the base.

        On the first call inside a SPRAYING run, snapshot the current
        /centre_point_3d (in camera_color_optical_frame: +X right, +Z fwd),
        compute the required displacement to reach (base_target_x, base_target_z)
        and turn it into a fixed-speed drive for each axis independently.
        Subsequent calls just replay that plan without feedback, so stale
        or missing /centre_point_3d updates don't stop the adjustment.

        Returns True once both axes have been driven for their planned time.
        """
        if not self._openloop_captured:
            if self.latest_centre_point is None:
                # Wait a short while for the first /centre_point_3d.
                if self.clock > self.base_align_capture_timeout:
                    self.get_logger().warn(
                        "Base align: no /centre_point_3d captured in "
                        f"{self.base_align_capture_timeout:.1f}s; skipping")
                    # Mark captured with zero plan so we fall through immediately.
                    self._openloop_v_x = 0.0
                    self._openloop_v_y = 0.0
                    self._openloop_t_x = 0.0
                    self._openloop_t_y = 0.0
                    self._openloop_captured = True
                    self._openloop_start_clock = self.clock
                return False
            obj_x = float(self.latest_centre_point.point.x)
            obj_z = float(self.latest_centre_point.point.z)
            disp_x = obj_x - self.base_target_x
            disp_z = obj_z - self.base_target_z
            speed = self.base_openloop_speed

            # Forward/back (camera +Z <-> base +linear.x).
            if abs(disp_z) > self.base_align_eps_z:
                self._openloop_v_x = speed if disp_z > 0 else -speed
                self._openloop_t_x = abs(disp_z) / speed
            else:
                self._openloop_v_x = 0.0
                self._openloop_t_x = 0.0

            # Strafe (camera +X = right <-> base -linear.y).
            if abs(disp_x) > self.base_align_eps_x:
                self._openloop_v_y = -speed if disp_x > 0 else speed
                self._openloop_t_y = abs(disp_x) / speed
            else:
                self._openloop_v_y = 0.0
                self._openloop_t_y = 0.0

            self._openloop_captured = True
            self._openloop_start_clock = self.clock
            self.get_logger().info(
                "Base align (open loop): "
                f"disp_x={disp_x:+.3f} m, disp_z={disp_z:+.3f} m -> "
                f"lin.x={self._openloop_v_x:+.3f} m/s for {self._openloop_t_x:.2f}s, "
                f"lin.y={self._openloop_v_y:+.3f} m/s for {self._openloop_t_y:.2f}s")

        elapsed = self.clock - self._openloop_start_clock
        if elapsed >= self._openloop_t_x and elapsed >= self._openloop_t_y:
            self._stop_base()
            return True

        twist = Twist()
        if elapsed < self._openloop_t_x:
            twist.linear.x = self._openloop_v_x
        if elapsed < self._openloop_t_y:
            twist.linear.y = self._openloop_v_y
        self.cmd_vel_publisher.publish(twist)
        return False


    def detection_callback(self, msg: Detection3DArray):

        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]

        if len(detection.results) == 0:
            return

        pose = detection.results[0].pose.pose

        # Create a PoseStamped for the bbox center
        box_pose = PoseStamped()
        box_pose.header = detection.header
        box_pose.pose = detection.bbox.center

        pose_stamped = PoseStamped()
        pose_stamped.header = detection.header
        pose_stamped.pose = pose

        self.latest_pose = pose_stamped
        self.latest_detection = msg
        self.latest_object_id = detection.results[0].hypothesis.class_id
        #print(f"latest object id: {self.latest_object_id}")
        self.latest_bbox_size = detection.bbox.size

    def object_callback(self, msg: String):
        self.latest_object_string = msg.data
        #print(f"latest object id from topic: {self.latest_object_id}")


    def state_machine_callback(self):
        global sensor_reading
        #if self.latest_detection is None:
        #    return  # No detections yet

        if self.state == States.IDLE:
            if self.state_transitioned:
                self.get_logger().info("Idle state: waiting for object detection and grab command...")
                self.state_transitioned = False  # Reset the flag after entering idle state
                home_joints = self.predefined_poses['home']['joint']
                traj_msg = JointTrajectory()
                traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(home_joints))]
                point = JointTrajectoryPoint()
                point.positions = home_joints
                point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
                traj_msg.points.append(point)
                self.robot_joint_publisher.publish(traj_msg)
                self.clock = 0.0
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 5.0:  # After 5 seconds in idle state, check for transition conditions
                    if self.transformed_pose is not None and self.grab_pressed:
                        self.state = States.GRABBING
                        self.get_logger().info("Transitioning to GRABBING state...")
                        self.grab_pressed = False  # Reset the flag
                        self.clock = 0.0  # Reset clock for next state
                        self.state_transitioned = True  # Set flag for next state
                    if self.spray_pressed:
                        self.state = States.SPRAYING
                        self.get_logger().info("Transitioning to SPRAYING state...")
                        self.spray_pressed = False  # Reset the flag
                        self.clock = 0.0  # Reset clock for next state
                        self.state_transitioned = True  # Set flag for next state

        if self.state == States.APPROACHING:
            if self.state_transitioned:
                self.get_logger().info("Approaching the object...")
                self.state_transitioned = False  # Reset the flag after approaching
                self.clock = 0.0  # Start counting time in approaching state
                self.object_pickup(self.transformed_pose)  # Implement this function to send the robot to the object
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 5.0:  # After 5 seconds in approaching state, transition to next state
                    self.state = States.GRABBING
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to GRABBING state...")
                    self.clock = 0.0  # Reset clock for next state

        if self.state == States.GRABBING:
            if self.state_transitioned:
                self.get_logger().info("Grabbing the object...")
                self.state_transitioned = False
                self.grab_done = False
                self.grab_succeeded = False
                self.clock = 0.0
                pick_msg = SimpleMove.Goal()
                pick_msg.target_pose = self.transformed_pose.pose
                pick_msg.target_pose.orientation.x = 0.0
                pick_msg.target_pose.orientation.y = -0.707
                pick_msg.target_pose.orientation.z = 0.0
                pick_msg.target_pose.orientation.w = 0.707
                pick_msg.move_gripper = 0
                pick_msg.object_attached = False
                send_goal_future = self.action_client.send_goal_async(
                    pick_msg,
                    feedback_callback=self.feedback_callback
                )
                send_goal_future.add_done_callback(self._on_grab_goal_response)
            else:
                if self.grab_done:
                    self.clock += 0.1
                    if self.grab_succeeded and self.clock > 2.0:  # brief wait for gripper to close
                        self.state = States.HOMING
                        self.state_transitioned = True
                        self.get_logger().info("Transitioning to HOMING state...")
                        self.clock = 0.0
                    elif not self.grab_succeeded:
                        self.state = States.IDLE
                        self.state_transitioned = True
                        self.get_logger().info("Grab failed, returning to IDLE state...")
                        self.clock = 0.0


        if self.state == States.PLACING:
            if self.state_transitioned:
                self.get_logger().info("Placing the object back...")
                self.state_transitioned = False  # Reset the flag after placing
                self.clock = 0.0  # Start counting time in placing state
                self.place_back_pressed = False  # Reset the flag
                self.place_notified = False  # One-shot guard for release notification
                place_msg = JointTrajectory()
                place_msg.joint_names = self.picked_joint_states.name[:-1]  # all but last joint
                point = JointTrajectoryPoint()
                point.positions = self.picked_joint_states.position[:-1]
                point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
                place_msg.points.append(point)
                self.get_logger().info(f"Placing back to joint states: {point.positions} with joint names: {place_msg.joint_names}")  # Print first 3 joint values for verification
                self.robot_joint_publisher.publish(place_msg)  # Move back to the joint states at the moment of picking
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 6.0:  # Open the gripper after 3 seconds in placing state
                    #self.get_logger().info("Opening gripper to release object...")
                    self.gripper_command(0.0)  # 0 = open gripper
                    if not self.place_notified:  # Notify the moment the object is released
                        self._notify_unity_done(self.place_back_client, "place back")
                        self.place_notified = True
                if self.clock > 10.0:  # After 10 seconds in placing state, transition to next state
                    self.state = States.IDLE
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to IDLE state...")
                    self.clock = 0.0  # Reset clock for next state
            # Implement placing logic here

        if self.state == States.HOMING:
            if self.state_transitioned:
                self.get_logger().info("Homing the robot...")
                home_joints = self.predefined_poses['home']['joint']
                self.get_logger().info(f"Home joints: {home_joints}")  # Print first 3 joint values for verification
                traj_msg = JointTrajectory()
                traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(home_joints))]
                point = JointTrajectoryPoint()
                point.positions = home_joints
                point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
                traj_msg.points.append(point)
                self.robot_joint_publisher.publish(traj_msg)
                self.state_transitioned = False  # Reset the flag after homing
                self.clock = 0.0  # Start counting time in homing state
            else:
                if self.clock == 5.0:  # Log at the moment of transition to next state
                    self.get_logger().info("Homing complete")
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 5.0:  # After 5 seconds in homing state, transition to next state
                    if self.pouch_pressed:
                        self.state = States.POUCHING
                        self.state_transitioned = True  # Set flag for next state
                        self.pouch_pressed = False  # Reset the flag
                        self.get_logger().info("Transitioning to POUCHING state...")
                        self.clock = 0.0  # Reset clock for next state
                    if self.sniff_pressed:
                        self.state = States.SNIFFING
                        self.state_transitioned = True  # Set flag for next state
                        self.sniff_pressed = False  # Reset the flag
                        self.get_logger().info("Transitioning to SNIFFING state...")
                        self.clock = 0.0  # Reset clock for next state
                    if self.place_back_pressed:
                        self.state = States.PLACING
                        self.state_transitioned = True  # Set flag for next state
                        self.place_back_pressed = False  # Reset the flag
                        self.get_logger().info("Transitioning to PLACING state...")
                        self.clock = 0.0  # Reset clock for next state
        

        if self.state == States.POUCHING:
            if self.state_transitioned:
                self.get_logger().info("Pouching the object...")
                pouch_joints = self.predefined_poses['pouch']['joint']
                traj_msg = JointTrajectory()
                traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(pouch_joints))]
                point = JointTrajectoryPoint()
                point.positions = pouch_joints
                point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
                traj_msg.points.append(point)
                self.robot_joint_publisher.publish(traj_msg)
                self.state_transitioned = False  # Reset the flag after pouching
                self.clock = 0.0  # Start counting time in pouching state
                self.pouch_notified = False  # One-shot guard for pouch notification
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 3.0:  # Open the gripper after 3 seconds in pouching state
                    #self.get_logger().info("Opening gripper to release object into pouch...")
                    self.gripper_command(0.0)  # 0 = open gripper
                    if not self.pouch_notified:  # Notify the moment the object drops in the pouch
                        self._notify_unity_done(self.pouch_object_client, "pouch")
                        self.pouch_notified = True
                if self.clock > 5.0:  # After 5 seconds in pouching state, transition to next state
                    self.state = States.IDLE
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to IDLE state...")
                    self.clock = 0.0  # Reset clock for next state

        if self.state == States.SNIFFING:
            if self.state_transitioned:
                sniff_joints = self.predefined_poses['sniff']['joint']
                traj_msg = JointTrajectory()
                traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(sniff_joints))]
                point = JointTrajectoryPoint()
                point.positions = sniff_joints
                point.time_from_start = rclpy.duration.Duration(seconds=3).to_msg()
                traj_msg.points.append(point)
                self.robot_joint_publisher.publish(traj_msg)
                self.state_transitioned = False  # Reset the flag after sniffing
                self.clock = 0.0  # Start counting time in sniffing state
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 5.0:  # After 5 seconds in sniffing state, transition to next state
                    threat_request = SetThreatLevel.Request()
                    threat_request.sensor_value = float(sensor_reading)
                    self.set_threat_client.call_async(threat_request)
                    self.get_logger().info(
                        f"Notified operator: sniff finished (sensor={sensor_reading})")
                    self.state = States.HOMING
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to HOMING state...")
                    self.clock = 0.0  # Reset clock for next state


        if self.state == States.SPRAYING:
            # Sub-sequence:
            #    0 -> align base to (base_target_x, base_target_z) via /cmd_vel
            #    1 -> pick_spray_approach
            #    2 -> pick_spray
            #    3 -> close gripper
            #    4 -> pick_spray_approach (lift bottle out of holder)
            #    5 -> transit to first spray pose (servo still 90)
            #    6 -> at first spray pose, servo -> 160, wait spray_start_delay
            #    7 -> iterate through remaining spray poses (servo=160)
            #    8 -> after last pose, servo -> 90, wait spray_stop_delay
            #    9 -> pick_spray_approach (start place-back)
            #   10 -> pick_spray
            #   11 -> open gripper
            #   12 -> pick_spray_approach (retreat)
            #   13 -> home (slow, big swing)
            #   14 -> transition to IDLE
            if self.state_transitioned:
                self.get_logger().info(
                    "Spraying sequence: starting open-loop base alignment "
                    f"(target x={self.base_target_x:.3f}, z={self.base_target_z:.3f} "
                    "in camera_color_optical_frame)")
                self.spray_substep = 0
                self.spray_pose_idx = 0
                self._openloop_captured = False
                self._openloop_start_clock = 0.0
                self.state_transitioned = False
                self.clock = 0.0
            else:
                self.clock += 0.1
                if self.spray_substep == 0:
                    if self._base_align_step():
                        self.get_logger().info(
                            "Spraying sequence: base aligned, moving to "
                            "pick_spray_approach")
                        self._publish_joint_traj(
                            self.predefined_poses['pick_spray_approach']['joint'],
                            duration=4.0)
                        self.spray_substep = 1
                        self.clock = 0.0
                    elif self.clock > self.base_align_timeout:
                        self.get_logger().warn(
                            f"Base alignment timed out after "
                            f"{self.base_align_timeout:.1f}s; proceeding anyway")
                        self._stop_base()
                        self._publish_joint_traj(
                            self.predefined_poses['pick_spray_approach']['joint'],
                            duration=4.0)
                        self.spray_substep = 1
                        self.clock = 0.0
                elif self.spray_substep == 1 and self.clock > 4.5:
                    self.get_logger().info(
                        "Spraying sequence: at approach, moving to pick_spray")
                    self._publish_joint_traj(
                        self.predefined_poses['pick_spray']['joint'],
                        duration=3.0)
                    self.spray_substep = 2
                    self.clock = 0.0
                elif self.spray_substep == 2 and self.clock > 3.5:
                    self.get_logger().info(
                        "Spraying sequence: at pick pose, closing gripper")
                    self.gripper_command(1.0)  # close
                    self._notify_unity_done(self.pick_spray_client, "spray can picked")
                    self.spray_substep = 3
                    self.clock = 0.0
                elif self.spray_substep == 3 and self.clock > 2.0:
                    self.get_logger().info(
                        "Spraying sequence: gripper closed, lifting bottle "
                        "to pick_spray_approach")
                    self._publish_joint_traj(
                        self.predefined_poses['pick_spray_approach']['joint'],
                        duration=3.0)
                    self.spray_substep = 4
                    self.clock = 0.0
                elif self.spray_substep == 4 and self.clock > 3.5:
                    if not self.spray_poses:
                        self.get_logger().error(
                            "No spray poses loaded; skipping to place-back")
                        self._publish_joint_traj(
                            self.predefined_poses['pick_spray_approach']['joint'],
                            duration=4.0)
                        self.spray_substep = 9
                        self.clock = 0.0
                    else:
                        self.get_logger().info(
                            "Spraying sequence: transiting to first spray pose "
                            f"(1/{len(self.spray_poses)})")
                        self.spray_pose_idx = 0
                        self._publish_joint_traj(
                            self.spray_poses[0],
                            duration=self.spray_pose_duration)
                        self.spray_substep = 5
                        self.clock = 0.0
                elif self.spray_substep == 5 and self.clock > self.spray_pose_settle:
                    # Arrived at first spray pose. Servo will read substep==6
                    # on the next tick and start pressing the trigger.
                    self.get_logger().info(
                        "Spraying sequence: at first spray pose, "
                        f"waiting {self.spray_start_delay:.1f}s for spray to flow "
                        "(servo -> 160)")
                    self.spray_substep = 6
                    self.clock = 0.0
                elif self.spray_substep == 6 and self.clock > self.spray_start_delay:
                    # Spray is flowing; move to the second pose (if any).
                    self.spray_pose_idx = 1
                    if self.spray_pose_idx >= len(self.spray_poses):
                        # Only one pose defined — jump straight to stop delay.
                        self.get_logger().info(
                            "Spraying sequence: single spray pose, moving to stop delay")
                        self.spray_substep = 8
                        self.clock = 0.0
                    else:
                        self.get_logger().info(
                            "Spraying sequence: start delay done, moving to spray pose "
                            f"{self.spray_pose_idx + 1}/{len(self.spray_poses)}")
                        self._publish_joint_traj(
                            self.spray_poses[self.spray_pose_idx],
                            duration=self.spray_pose_duration)
                        self.spray_substep = 7
                        self.clock = 0.0
                elif self.spray_substep == 7 and self.clock > self.spray_pose_settle:
                    self.spray_pose_idx += 1
                    if self.spray_pose_idx >= len(self.spray_poses):
                        # At last pose. Servo will go back to 90 when
                        # substep == 8 is observed on the next tick.
                        self.get_logger().info(
                            "Spraying sequence: last spray pose reached, "
                            f"waiting {self.spray_stop_delay:.1f}s for spray to stop "
                            "(servo -> 90)")
                        self.spray_substep = 8
                        self.clock = 0.0
                    else:
                        self.get_logger().info(
                            "Spraying sequence: moving to spray pose "
                            f"{self.spray_pose_idx + 1}/{len(self.spray_poses)}")
                        self._publish_joint_traj(
                            self.spray_poses[self.spray_pose_idx],
                            duration=self.spray_pose_duration)
                        self.clock = 0.0
                elif self.spray_substep == 8 and self.clock > self.spray_stop_delay:
                    self.get_logger().info(
                        "Spraying sequence: spray stopped, starting place-back "
                        "(-> pick_spray_approach)")
                    self._notify_unity_done(self.spray_client, "spray")
                    self._publish_joint_traj(
                        self.predefined_poses['pick_spray_approach']['joint'],
                        duration=4.0)
                    self.spray_substep = 9
                    self.clock = 0.0
                elif self.spray_substep == 9 and self.clock > 4.5:
                    self.get_logger().info(
                        "Spraying sequence: at approach, lowering to pick_spray "
                        "to place bottle")
                    self._publish_joint_traj(
                        self.predefined_poses['pick_spray']['joint'],
                        duration=3.0)
                    self.spray_substep = 10
                    self.clock = 0.0
                elif self.spray_substep == 10 and self.clock > 3.5:
                    self.get_logger().info(
                        "Spraying sequence: at pick pose, releasing bottle "
                        "(opening gripper)")
                    self.gripper_command(0.0)  # open
                    self._notify_unity_done(self.place_spray_client, "spray can released")
                    self.spray_substep = 11
                    self.clock = 0.0
                elif self.spray_substep == 11 and self.clock > 2.0:
                    self.get_logger().info(
                        "Spraying sequence: gripper open, retreating to "
                        "pick_spray_approach")
                    self._publish_joint_traj(
                        self.predefined_poses['pick_spray_approach']['joint'],
                        duration=3.0)
                    self.spray_substep = 12
                    self.clock = 0.0
                elif self.spray_substep == 12 and self.clock > 3.5:
                    # Big swing on joint 1; use a slow trajectory so the
                    # controller doesn't reject it on velocity limits.
                    self.get_logger().info(
                        "Spraying sequence: retreat complete, returning to home")
                    self._publish_joint_traj(
                        self.predefined_poses['home']['joint'],
                        duration=5.0)
                    self.spray_substep = 13
                    self.clock = 0.0
                elif self.spray_substep == 13 and self.clock > 5.5:
                    self.get_logger().info(
                        "Spraying sequence complete; returning to IDLE")
                    self.state = States.IDLE
                    self.state_transitioned = True
                    self.clock = 0.0



        if self.state == States.NAVIGATING: # currently not implemented
            self.get_logger().info("Navigating to target location...")
            # Implement navigation logic here


    def feedback_callback(self, feedback):
        pass

    def _on_grab_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Grab goal rejected by action server.")
            self.grab_done = True
            self.grab_succeeded = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_grab_result)

    def _on_grab_result(self, future):
        result = future.result()
        self.grab_succeeded = result.result.success
        self.grab_done = True
        if self.grab_succeeded:
            self.get_logger().info("Grab action succeeded.")
            self.gripper_command(1.0)
            self.picked_joint_states = self.latest_joint_states
        else:
            self.get_logger().error("Grab action failed.")

        # Notify the operator whether the grab succeeded or not.
        pick_request = SetBool.Request()
        pick_request.data = self.grab_succeeded
        self.pick_object_client.call_async(pick_request)
        self.get_logger().info(
            f"Notified operator: grab {'succeeded' if self.grab_succeeded else 'failed'}")



def main(args=None):
    rclpy.init(args=args)
    node = KinovaStateMachineNode()
    sensor_node = SensorReaderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sensor_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()