#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

class checkpoints_to_follow(Node):

    def __init__(self):
        super().__init__('checkpoints_contour')
        
        self.navigator = BasicNavigator()

        self.goal_pose0 = self.create_pose_stamped(2.1, 0.9, 3.1)
        self.goal_pose1 = self.create_pose_stamped(1.2, 1.0, 3.1)
        self.goal_pose2 = self.create_pose_stamped(0.95, 1.0, 3.1)
        self.goal_pose3 = self.create_pose_stamped(-1.15, 1.10, 3.0)
        self.goal_pose4 = self.create_pose_stamped(-2.8, 1.25, 4.7)
        self.goal_pose5 = self.create_pose_stamped(-3.0, 0.25, 4.7)
        self.goal_pose6 = self.create_pose_stamped(-3.0, -0.25, 4.7)
        self.goal_pose7 = self.create_pose_stamped(-2.7, -0.85, 0.0)
        self.goal_pose8 = self.create_pose_stamped(-1.7,-0.8, 0.0)
        self.goal_pose9 = self.create_pose_stamped(-0.5,-0.8, 1.57)
        self.goal_pose10 = self.create_pose_stamped(1.5,-0.9, 0.3)
        self.goal_pose11 = self.create_pose_stamped(2.5,-0.9, 1.5)
        self.goal_pose12 = self.create_pose_stamped(2.6,-0.6, 3.1)
        self.goal_pose13 = self.create_pose_stamped(3.0,0.3, 1.5)
        self.goal_pose14 = self.create_pose_stamped(3.0,0.9, 3.1)
        self.goal_pose15 = self.create_pose_stamped(-2.5,0.9, 4.7)
        self.goal_pose16 = self.create_pose_stamped(-2.6,-0.5, 0.3)
        self.goal_pose17 = self.create_pose_stamped(2.1,-0.7, 1.5)
        self.goal_pose18 = self.create_pose_stamped(2.2,0.3, 3.1)
        self.goal_pose19 = self.create_pose_stamped(-2.1,0.6, 4.7)
        self.goal_pose20 = self.create_pose_stamped(-2.1, -0.4, 0.0)
        self.goal_pose21 = self.create_pose_stamped(1.7, -0.4, 1.5)
        self.goal_pose22 = self.create_pose_stamped(1.7, 0.2, 1.5)
        self.goal_pose23 = self.create_pose_stamped(-1.8, 0.0, 4.7)
        self.goal_pose24 = self.create_pose_stamped(-1.8, -0.2, 0.0)
        self.goal_pose25 = self.create_pose_stamped(1.25, -0.4, 1.5)
        self.goal_pose26 = self.create_pose_stamped(1.25, 0.2, 2.5)
        self.goal_pose27 = self.create_pose_stamped(-1.6, 0.2, 2.5)
        

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
        waypoints = [self.goal_pose0 , self.goal_pose1, self.goal_pose2, self.goal_pose3, self.goal_pose4, self.goal_pose5, self.goal_pose6, self.goal_pose7, self.goal_pose8, self.goal_pose9, self.goal_pose10, self.goal_pose11, self.goal_pose12, self.goal_pose13, self.goal_pose14, self.goal_pose15, self.goal_pose16, self.goal_pose17, self.goal_pose18, self.goal_pose19, self.goal_pose20, self.goal_pose21, self.goal_pose22, self.goal_pose23, self.goal_pose24, self.goal_pose25, self.goal_pose26, self.goal_pose27]
        self.get_logger().info("Following waypoints...")
        self.navigator.followWaypoints(waypoints)
        self.check_task_complete()

    def go_to_waypoint(self):
        self.get_logger().info("Navigating to goal 1...")
        self.navigator.goToPose(self.goal_pose7_1)
        self.check_task_complete()

    def check_task_complete(self):
        """Check if the navigation task is complete"""
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(feedback)
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
