import rclpy
from rclpy.node import Node
import time
import yaml
from pathlib import Path

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_msgs.msg import Float32, String, Int32

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from move_kinova_msgs.action import SimpleMove  

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


class KinovaActionNode(Node):
    def __init__(self):
        super().__init__('kinova_action_node')

        # ---- Subscriber ----
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/output_with_class',
            self.detection_callback,
            10
        )

        self.object_point_subscription = self.create_subscription(
            PointStamped,
            '/centre_point_3d',
            self.object_point_callback,
            10
        )

        self.object_subscription = self.create_subscription(
            String,
            '/current_object',
            self.object_callback,
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

        self.pick_object_client = self.create_client(PickObject, '/unity_pick_object')
        self.pouch_object_client = self.create_client(SetBool, '/unity_pouch_object')
        self.place_back_client = self.create_client(SetBool, '/unity_place_back_object')
        self.set_threat_client = self.create_client(SetThreatLevel, '/unity_set_threat_level')
        self.robot_joint_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # ---- Action Client ----
        self._moveit_client = ActionClient(self, MoveGroup, '/move_action')

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
        self.latest_bbox_size = None
        self.transformed_pose = None
        self.picked_pose = None
        self.latest_detection = None
        self.picking_object = True
        self.state = States.IDLE
        self.state_transitioned = True
        self.pouch_pressed = False
        self.grab_pressed = False
        self.sniff_pressed = False
        self.clock = 0.0

        # read poses from yaml
        yaml_path = Path(__file__).parent / 'config' / 'robot_poses.yaml' # in config folder
        with open(yaml_path, 'r') as file:
            self.predefined_poses = yaml.safe_load(file)
            print(f"Loaded predefined poses: {self.predefined_poses.get('home', 'Not found').get('joint', 'Not found')[:3]}")


    def object_point_callback(self, msg: PointStamped):
        self.latest_pose = msg
        try:
            # Transform the point to the robot's base frame but point itself cannot be tranformed, so we create a PoseStamped with the point as the position and a default orientation
            pose_msg = Pose()
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


    def gripper_command(self, command):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = command  # 0 for open, 1 for close
        self.gripper_client.send_goal_async(goal_msg)


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

    def object_pickup(self, pose):
        if not self._moveit_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt action server not available!")
            return
        self.get_logger().info("Sending goal to MoveIt...")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        # Set the target pose for the end effector where end effector is with fixed orientation, only position is from the detection
        goal_msg.request.goal_constraints.append(Constraints())
        goal_msg.request.goal_constraints[0].position_constraints.append(PositionConstraint())
        pos_constraint = goal_msg.request.goal_constraints[0].position_constraints[0]
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "end_effector_link"
        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        pos_constraint.constraint_region.primitives[0].dimensions = [0.001, 0.001, 0.001]
        pos_constraint.constraint_region.primitive_poses.append(pose)
        pos_constraint.weight = 1.0
        orientation_constraint = OrientationConstraint() #orientation is fixed, can be adjusted if needed
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "end_effector_link"
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        goal_msg.request.goal_constraints[0].orientation_constraints.append(orientation_constraint)
        self._moveit_client.send_goal_async(goal_msg)




    def state_machine_callback(self):
        global sensor_reading
        #if self.latest_detection is None:
        #    return  # No detections yet

        if self.state == States.IDLE:
            if self.transformed_pose is not None and self.grab_pressed:
                self.state = States.APPROACHING
                self.get_logger().info("Transitioning to APPROACHING state...")
                self.grab_pressed = False  # Reset the flag

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
                self.state_transitioned = False  # Reset the flag after grabbing
                self.clock = 0.0  # Start counting time in grabbing state
                self.action_client.wait_for_server()
                goal_msg = SimpleMove.Goal()
                goal_msg.target_pose = self.transformed_pose
                goal_msg.target_pose.pose.position.z += 0.1  # Move down to the object
                self.action_client.send_goal_async(goal_msg)
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 3.0:  # Close the gripper after 3 seconds in grabbing state
                    #self.get_logger().info("Closing gripper to grab object...")
                    self.gripper_command(1.0)  # 1 = close gripper
                if self.clock > 5.0:  # After 5 seconds in grabbing state, transition to next state
                    self.state = States.HOMEING
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to HOMEING state...")
                    self.clock = 0.0  # Reset clock for next state
                # Implement grabbing logic here


        if self.state == States.PLACING:
            self.get_logger().info("Placing the object back...")
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
                point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
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
        

        if self.state == States.POUCHING:
            if self.state_transitioned:
                self.get_logger().info("Pouching the object...")
                pouch_joints = self.predefined_poses['pouch']['joint']
                traj_msg = JointTrajectory()
                traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(pouch_joints))]
                point = JointTrajectoryPoint()
                point.positions = pouch_joints
                point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
                traj_msg.points.append(point)
                self.robot_joint_publisher.publish(traj_msg)
                self.state_transitioned = False  # Reset the flag after pouching
                self.clock = 0.0  # Start counting time in pouching state
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 3.0:  # Open the gripper after 3 seconds in pouching state
                    #self.get_logger().info("Opening gripper to release object into pouch...")
                    self.gripper_command(0.0)  # 0 = open gripper
                if self.clock > 5.0:  # After 5 seconds in pouching state, transition to next state
                    self.state = States.HOMING
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to HOMING state...")
                    self.clock = 0.0  # Reset clock for next state

        if self.state == States.SNIFFING:
            if self.state_transitioned:
                sniff_joints = self.predefined_poses['sniff']['joint']
                traj_msg = JointTrajectory()
                traj_msg.joint_names = [f'joint_{i+1}' for i in range(len(sniff_joints))]
                point = JointTrajectoryPoint()
                point.positions = sniff_joints
                point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
                traj_msg.points.append(point)
                self.robot_joint_publisher.publish(traj_msg)
                self.state_transitioned = False  # Reset the flag after sniffing
            else:
                self.clock += 0.1  # Increment clock by timer period
                if self.clock > 5.0:  # After 5 seconds in sniffing state, transition to next state
                    self.state = States.HOMING
                    self.state_transitioned = True  # Set flag for next state
                    self.get_logger().info("Transitioning to HOMING state...")
                    self.clock = 0.0  # Reset clock for next state


        if self.state == States.SPRAYING:
            self.get_logger().info("Spraying the object...")
            # Implement spraying logic here

        if self.state == States.SNIFFING:
            self.get_logger().info("Sniffing the object...")
            # Implement sniffing logic here



        if self.state == States.NAVIGATING:
            self.get_logger().info("Navigating to target location...")
            # Implement navigation logic here



def main(args=None):
    rclpy.init(args=args)
    node = KinovaActionNode()
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