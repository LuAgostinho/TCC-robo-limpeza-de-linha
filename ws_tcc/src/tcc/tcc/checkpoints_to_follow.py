#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

class checkpoints_to_follow(Node):

    def __init__(self):
        super().__init__('checkpoints_to_follow')
        
        self.navigator = BasicNavigator()

        self.goal_pose0 = self.create_pose_stamped(2.2, 1.05, 3.14)
        self.goal_pose1 = self.create_pose_stamped(1.2, 1.0, 3.14)
        self.goal_pose2 = self.create_pose_stamped(0.95, 1.0, 3.14)
        self.goal_pose3 = self.create_pose_stamped(-0.85, 1.15, 3.14)
        self.goal_pose4 = self.create_pose_stamped(-3.0, 1.25, 0.0)

        self.follow_waypoints()

        #self.go_to_waypoint()

    def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w= q_w
        return pose

    def follow_waypoints(self):
        waypoints = [self.goal_pose0 ,self.goal_pose1,self.goal_pose2 ,self.goal_pose3, self.goal_pose4]
        self.get_logger().info("Following waypoints...")
        self.navigator.followWaypoints(waypoints)
        self.check_task_complete()

    def go_to_waypoint(self):
        self.get_logger().info("Navigating to goal 1...")
        self.navigator.goToPose(self.goal_pose1)
        self.check_task_complete()

    def check_task_complete(self):
        """Check if the navigation task is complete"""
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
        result = self.navigator.getResult()

def main(args=None):
    #init
    rclpy.init(args=args)
    node = checkpoints_to_follow()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown node
        node.navigator.getLogger().info("Shutting down checkpoint node...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
