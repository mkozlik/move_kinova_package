import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from rclpy.action import ActionClient
from move_kinova_msgs.action import SimpleMove  # <-- replace with your action

import tf2_ros
import tf2_geometry_msgs


class GrabServiceNode(Node):

    def __init__(self):
        super().__init__('grab_service_node')

        # ---- Subscriber ----
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # ---- Service ----
        self.service = self.create_service(
            Trigger,
            '/trigger_grab',
            self.service_callback
        )

        # ---- Action Client ----
        self.action_client = ActionClient(
            self,
            SimpleMove,
            'grab_object'
        )

        # ---- TF Buffer ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for latest pose
        self.latest_pose = None

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

        pose_stamped = PoseStamped()
        pose_stamped.header = detection.header
        pose_stamped.pose = pose

        self.latest_pose = pose_stamped

    # ==================================================
    # Service
    # ==================================================
    def service_callback(self, request, response):

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
        except Exception as e:
            self.get_logger().error(str(e))
            response.success = False
            response.message = "TF transform failed"
            return response

        self.send_action_goal(transformed_pose)

        response.success = True
        response.message = "Grab goal sent"
        return response

    # ==================================================
    # Action
    # ==================================================
    def send_action_goal(self, pose_stamped):

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return

        goal_msg = SimpleMove.Goal()

        # Your action expects geometry_msgs/Pose
        goal_msg.target_pose = pose_stamped.pose

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Status: {feedback.status}")

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