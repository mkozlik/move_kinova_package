import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from move_kinova_msgs.action import SimpleMove
import asyncio

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from hero_custom_msgs.srv import PickObject, ReleaseObject


class KinovaBridge(Node):
    def __init__(self):
        super().__init__('kinova_bridge_node')
        
        # Client to talk to MoveIt's internal action
        self._moveit_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')
        self.planning_scene_publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.grab_service_client = self.create_client(PickObject, '/pick_object')

        
        # Server for your simplified terminal command
        self._action_server = ActionServer(
            self, SimpleMove, 'grab_object', self.execute_callback)
        
        self.get_logger().info("Kinova Bridge Node is online.")


    def setup_scene_collisions(self, pos_x, pos_y, pos_z, size_x, size_y, size_z, name="detected_object"):
        # Create a collision object based on the latest detection
        collision_object = CollisionObject()
        collision_object.id = name
        collision_object.header.frame_id = "world"
        collision_object.primitives = []
        collision_object.primitive_poses = []
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            size_x, 
            size_y,
            size_z
        ]
        pose = Pose()
        pose.position.x = pos_x
        pose.position.y = pos_y
        pose.position.z = pos_z
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        
        # Publish the collision object to the planning scene
        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.is_diff = True # Crucial: only adds the change
        
        self.planning_scene_publisher.publish(planning_scene_msg)

    async def execute_callback(self, goal_handle):
        pos_x = -0.345
        pos_y = 0.0
        pos_z = 0.09

        size_x = 0.23
        size_y = 0.33
        size_z = 0.18
        self.setup_scene_collisions(pos_x, pos_y, pos_z, size_x, size_y, size_z, name="sensor_box")

        pos_x = 0.125
        pos_y = 0.0
        pos_z = 0.04

        size_x = 0.08
        size_y = 0.08
        size_z = 0.08
        self.setup_scene_collisions(pos_x, pos_y, pos_z, size_x, size_y, size_z, name="lidar")

        target = goal_handle.request.target_pose
        gripper_command = goal_handle.request.move_gripper
        
        # 1. Wait for MoveIt
        if not self._moveit_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            return SimpleMove.Result(success=False)

        # 2. Build the complex MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        # Position Constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "world"
        pos_con.link_name = "end_effector_link" # Verify this name!
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]
        
        volume = BoundingVolume()
        volume.primitives.append(box)
        volume.primitive_poses.append(target)
        
        pos_con.constraint_region = volume
        pos_con.weight = 1.0
        
        # Orientation Constraint
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "world"
        ori_con.link_name = "end_effector_link"
        ori_con.orientation = target.orientation
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        
        goal_msg.request.goal_constraints.append(Constraints(
            position_constraints=[pos_con], 
            orientation_constraints=[ori_con]
        ))

        # 3. Send and Wait
        self.get_logger().info("Sending goal to MoveIt...")
        send_goal_future = await self._moveit_client.send_goal_async(goal_msg)
        
        if not send_goal_future.accepted:
            goal_handle.abort()
            return SimpleMove.Result(success=False)

        # Wait for arm motion result
        move_result = await send_goal_future.get_result_async()

        if move_result.status != 4:  # 4 = SUCCEEDED
            self.get_logger().error("Arm motion failed.")
            goal_handle.abort()
            return SimpleMove.Result(success=False)

        self.get_logger().info("Arm motion completed. Waiting 2 seconds...")
        #await asyncio.sleep(2)

        # 4. Send Gripper Command
        # Wait for gripper server

        if gripper_command == 1:
            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Gripper action server not available!")
                goal_handle.abort()
                return SimpleMove.Result(success=False)

            # Create close command
            gripper_goal = GripperCommand.Goal()
            gripper_goal.command.position = 1.0   # 0.0 = fully closed (for most Kinova grippers)
            gripper_goal.command.max_effort = 50.0

            self.get_logger().info("Closing gripper...")
            gripper_future = await self.gripper_client.send_goal_async(gripper_goal)

            await gripper_future.get_result_async()  # Wait for gripper action to complete

            if not gripper_future.accepted:
                self.get_logger().error("Gripper goal rejected.")
                goal_handle.abort()
                return SimpleMove.Result(success=False)

            gripper_result_future = await gripper_future.get_result_async()
            gripper_result = gripper_result_future.result

            closed_position = gripper_result.position
            self.get_logger().info(f"Gripper closed to position: {closed_position}")

            if closed_position > 0.65:
                self.get_logger().warn("No object detected. Re-opening gripper...")

                open_goal = GripperCommand.Goal()
                open_goal.command.position = 0.0   # Fully open
                open_goal.command.max_effort = 50.0

                await self.gripper_client.send_goal_async(open_goal)
                
                goal_handle.abort()
                return SimpleMove.Result(success=False)

            self.get_logger().info("Object successfully grasped!")

        if gripper_command == 2: # relase object
            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Gripper action server not available!")
                goal_handle.abort()
                return SimpleMove.Result(success=False)

            # Create open command
            gripper_goal = GripperCommand.Goal()
            gripper_goal.command.position = 0.0   # 0.0 = fully open (for most Kinova grippers)
            gripper_goal.command.max_effort = 50.0

            self.get_logger().info("Opening gripper...")
            gripper_future = await self.gripper_client.send_goal_async(gripper_goal)

            await gripper_future.get_result_async()  # Wait for gripper action to complete

            if not gripper_future.accepted:
                self.get_logger().error("Gripper goal rejected.")
                goal_handle.abort()
                return SimpleMove.Result(success=False)

            self.get_logger().info("Object released.")
            
        goal_handle.succeed()
        return SimpleMove.Result(success=True)


def main():
    rclpy.init()
    rclpy.spin(KinovaBridge())
    rclpy.shutdown()