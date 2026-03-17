import rclpy
from rclpy.node import Node
import time

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_msgs.msg import Float32

from rclpy.action import ActionClient
from move_kinova_msgs.action import SimpleMove  # <-- replace with your action

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from hero_custom_msgs.srv import PickObject, ReleaseObject, SetThreatLevel

import tf2_ros
import tf2_geometry_msgs


class GrabServiceNode(Node):

    def __init__(self):
        super().__init__('grab_service_node')

        # ---- Subscriber ----
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/output',
            self.detection_callback,
            10
        )

        self.sensor_sub = self.create_subscription(
            Float32,
            '/poly5_topic',
            self.sensor_callback,
            10
        )

        # ---- Service ----
        self.grab_service = self.create_service(
            SetBool,
            '/trigger_grab',
            self.grab_service_callback
        )

        self.release_service = self.create_service(
            SetBool,
            '/trigger_release',
            self.release_service_callback
        )

        self.place_back_service = self.create_service(
            SetBool,
            '/trigger_place_back',
            self.place_back_service_callback
        )

        self.pick_object_client = self.create_client(PickObject, '/pick_object')
        self.release_object_client = self.create_client(ReleaseObject, '/release_object')
        self.set_threat_client = self.create_client(SetThreatLevel, '/set_threat_level')

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

        # ---- TF Buffer ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sensor_reading = None

        # Storage for latest pose
        self.latest_pose = None
        self.latest_bbox_size = None

        self.picked_pose = None

    # ==================================================
    # Subscriber
    # ==================================================
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
        self.latest_bbox_size = detection.bbox.size


    def sensor_callback(self, msg: Float32):
        self.sensor_reading = msg.data
        #self.get_logger().info(f"Received sensor reading: {self.sensor_reading}")

    # ==================================================
    # Service
    # ==================================================
    def grab_service_callback(self, request, response):
        if self.latest_pose is None:
            response.success = False
            response.message = "No detection available"
            return response

        try:
            transformed_pose = self.tf_buffer.transform(
                self.latest_pose,
                'world',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Update the MoveIt collision environment
            self.remove_object_from_scene("table_obstacle")  # Clear previous object if exists
            self.publish_planning_scene_object(transformed_pose)
            self.publish_table_as_obstacle(transformed_pose)
            
            # Send the goal to the robot
            self.send_action_goal(transformed_pose)

            response.success = True
            response.message = "Object added and Grab goal sent"
            self.picked_pose = transformed_pose  # Store the pose for later use


        except Exception as e:
            self.get_logger().error(f"Failed: {str(e)}")
            response.success = False
            response.message = "Error in processing"
            
        return response

    def release_service_callback(self, request, response):
        try:
            # For demonstration, let's just log the release action
            self.get_logger().info("Release service called")
            self.send_release_action_goal()  # Implement this method to send a release command to the robot
        except Exception as e:
            self.get_logger().error(f"Release failed: {str(e)}")
            response.success = False
            response.message = "Error in release"
            return response

        response.success = True
        response.message = "Release triggered"
        return response

    def place_back_service_callback(self, request, response):
        try:
            self.publish_table_as_obstacle(self.picked_pose)  # Re-add the table obstacle to the planning scene
            self.get_logger().info("Place back service called")
            self.send_place_back_action_goal(self.picked_pose)
              # Implement this method to send a command to move back to home position
        except Exception as e:
            self.get_logger().error(f"Place back failed: {str(e)}")
            response.success = False
            response.message = "Error in placing back"
            return response

        response.success = True
        response.message = "Place back triggered"
        return response


    # ==================================================
    # Action
    # ==================================================
    def send_action_goal(self, pose_stamped):

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return

        #self.send_action_above_goal(pose_stamped)  # Move above the object first

        #time.sleep(1)  # Wait for the robot to move above the object

        pick_msg = SimpleMove.Goal()

        # Your action expects geometry_msgs/Pose
        pick_msg.target_pose = pose_stamped.pose

        pick_msg.target_pose.position.x -= 0.1 # hacky way to not colide with the object

        pick_msg.target_pose.orientation.x = 0.707
        pick_msg.target_pose.orientation.y = 0.0
        pick_msg.target_pose.orientation.z = 0.707
        pick_msg.target_pose.orientation.w = 0.0
        pick_msg.move_gripper = 1

        send_goal_future = self.action_client.send_goal_async(
            pick_msg,
            feedback_callback=self.feedback_callback
        )

        #self.send_action_above_goal(pose_stamped)  # Move above the object first
        #time.sleep(1)  # Wait for the robot to move above the object

        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result success: {result.success}")
        self.remove_object_from_scene('detected_object')

        self.return_home_action_goal()  # Move back to home position after picking
        time.sleep(4)  # Wait for the robot to move back home

        request = PickObject.Request()
        response = PickObject.Response()
        request.id = 1000 # Placeholder ID, adjust as needed
        if result.success:
            response.success = True
            response.message = "Object picked successfully"
            self.pick_object_client.call_async(request)
            time.sleep(4)
            self.send_sniff_action_goal()
        else:
            response.success = False
            response.message = "Failed to pick object"
            self.pick_object_client.call_async(request)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Status: {feedback.status}")

    # --------------move above object action----------------

    def send_action_above_goal(self, pose_stamped):

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return

        pick_msg = SimpleMove.Goal()

        # Your action expects geometry_msgs/Pose
        pick_msg.target_pose = pose_stamped.pose

        pick_msg.target_pose.position.x -= 0.1 # hacky way to not colide with the object
        pick_msg.target_pose.position.z += 0.1 # Move above the object

        pick_msg.target_pose.orientation.x = 0.707
        pick_msg.target_pose.orientation.y = 0.0
        pick_msg.target_pose.orientation.z = 0.707
        pick_msg.target_pose.orientation.w = 0.0
        pick_msg.move_gripper = 0

        send_goal_future = self.action_client.send_goal_async(
            pick_msg,
            feedback_callback=self.feedback_callback
        )


    #---------------sniff action----------------


    def send_sniff_action_goal(self):
        sniff_msg = SimpleMove.Goal()
        sniff_msg.target_pose.position.x = -0.02
        sniff_msg.target_pose.position.y = -0.35
        sniff_msg.target_pose.position.z = 0.1
        sniff_msg.target_pose.orientation.x = 0.0
        sniff_msg.target_pose.orientation.y = -0.707
        sniff_msg.target_pose.orientation.z = 0.0
        sniff_msg.target_pose.orientation.w = 0.707
        sniff_msg.move_gripper = 0

        send_sniff_future = self.action_client.send_goal_async(
            sniff_msg,
            feedback_callback=self.sniff_feedback_callback
        )

        send_sniff_future.add_done_callback(self.sniff_response_callback)
        
    def sniff_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Sniff goal rejected")
            return

        self.get_logger().info("Sniff goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.sniff_result_react)

    def sniff_result_react(self, future):
        result = future.result().result
        self.get_logger().info(f"Result success: {result.success}")
        time.sleep(10)  # Wait for sensor reading to update
        threat_request = SetThreatLevel.Request()
        threat_request.sensor_value = self.sensor_reading  # Example threat level
        self.set_threat_client.call_async(threat_request)
        self.get_logger().info(f"Sensor reading at sniff: {self.sensor_reading}")
        self.remove_object_from_scene('table_obstacle')
        self.return_home_action_goal()


    def sniff_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Status: {feedback.status}")

    #---------------return home action----------------

    def return_home_action_goal(self):
        home_msg = SimpleMove.Goal()
        home_msg.target_pose.position.x = 0.2
        home_msg.target_pose.position.y = 0.0
        home_msg.target_pose.position.z = 0.5
        home_msg.target_pose.orientation.x = 0.707
        home_msg.target_pose.orientation.y = 0.0
        home_msg.target_pose.orientation.z = 0.707
        home_msg.target_pose.orientation.w = 0.0
        home_msg.move_gripper = 0

        send_home_future = self.action_client.send_goal_async(
            home_msg,
            feedback_callback=self.home_feedback_callback
        )

        send_home_future.add_done_callback(self.home_response_callback)

    def home_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Home goal rejected")
            return

        self.get_logger().info("Home goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Returned home, success: {result.success}")

    def home_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Status: {feedback.status}")


    #---------------release action----------------

    def send_release_action_goal(self):
        release_msg = SimpleMove.Goal()
        release_msg.target_pose.position.x = 0.4
        release_msg.target_pose.position.y = 0.1
        release_msg.target_pose.position.z = -0.2
        release_msg.target_pose.orientation.x = 0.866
        release_msg.target_pose.orientation.y = 0.0
        release_msg.target_pose.orientation.z = 0.5
        release_msg.target_pose.orientation.w = 0.0
        release_msg.move_gripper = 2  # Custom flag to indicate release

        send_release_future = self.action_client.send_goal_async(
            release_msg,
            feedback_callback=self.release_feedback_callback
        )

        send_release_future.add_done_callback(self.release_response_callback)

    def release_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Release goal rejected")
            return

        self.get_logger().info("Release goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.release_result_callback)

    def release_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Release action completed, success: {result.success}")
        time.sleep(2)  # Wait for any final movements to complete
        release_request = ReleaseObject.Request()
        release_request.id = 1000 # Placeholder ID, adjust as needed
        release_response = ReleaseObject.Response()
        release_response.success = result.success
        release_response.message = "Object released" if result.success else "Failed to release object"
        self.release_object_client.call_async(release_request)  # Notify that release is done
        self.return_home_action_goal()

    def release_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Status: {feedback.status}")


    #-----------------place back action----------------

    def send_place_back_action_goal(self, pose_stamped):

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return

        #self.send_action_above_goal(pose_stamped)  # Move above the object first

        #time.sleep(1)  # Wait for the robot to move above the object

        pick_msg = SimpleMove.Goal()

        # Your action expects geometry_msgs/Pose
        pick_msg.target_pose = pose_stamped.pose


        pick_msg.target_pose.orientation.x = 0.707
        pick_msg.target_pose.orientation.y = 0.0
        pick_msg.target_pose.orientation.z = 0.707
        pick_msg.target_pose.orientation.w = 0.0
        pick_msg.move_gripper = 2

        send_goal_future = self.action_client.send_goal_async(
            pick_msg,
            feedback_callback=self.feedback_callback
        )

        #self.send_action_above_goal(pose_stamped)  # Move above the object first
        #time.sleep(1)  # Wait for the robot to move above the object

        send_goal_future.add_done_callback(self.goal_place_back_response_callback)

    def goal_place_back_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Place back goal rejected")
            return

        self.get_logger().info("Place back goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.place_back_result_callback)

    def place_back_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Place back completed, success: {result.success}")
        self.return_home_action_goal()  # Move back to home position after placing back


    # ==================================================
    # Publisher for Planning Scene (to add detected object as collision object)
    # ==================================================

    def publish_planning_scene_object(self, transformed_pose):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'detected_object'
        
        # Define the box shape
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            self.latest_bbox_size.x/4, 
            self.latest_bbox_size.y,  ## y is height for some reason, so we keep it as is
            self.latest_bbox_size.z/4
        ]

        collision_object.primitives.append(box)
        transformed_pose.pose.position.y -= 0.05
        collision_object.primitive_poses.append(transformed_pose.pose)
        collision_object.operation = CollisionObject.ADD

        # Wrap in PlanningScene message
        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.is_diff = True # Crucial: only adds the change
        
        self.planning_scene_publisher.publish(planning_scene_msg)
        self.get_logger().info("Object added to planning scene.")

    def publish_table_as_obstacle(self, transformed_pose):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'table_obstacle'
        
        # Define the box shape for the table
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.15, 0.15, 0.01]  # Example dimensions for the table

        collision_object.primitives.append(box)
        
        # Position the table at the transformed pose (adjust as needed)
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = transformed_pose.pose.position.x
        table_pose.pose.position.y = transformed_pose.pose.position.y
        table_pose.pose.position.z = transformed_pose.pose.position.z - 0.1  # Slightly below the object
        table_pose.pose.orientation.w = 1.0  # No rotation

        collision_object.primitive_poses.append(table_pose.pose)
        collision_object.operation = CollisionObject.ADD

        # Wrap in PlanningScene message
        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.is_diff = True
        
        self.planning_scene_publisher.publish(planning_scene_msg)
        self.get_logger().info("Table obstacle added to planning scene.")


    def remove_object_from_scene(self, object_id='detected_object'):
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.is_diff = True
        
        self.planning_scene_publisher.publish(planning_scene_msg)
        self.get_logger().info(f"Object '{object_id}' removed from planning scene.")

    # ==================================================
    # Main
    # ==================================================
def main(args=None):
    rclpy.init(args=args)
    node = GrabServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()