#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)


class checkpoints_to_follow(Node):

    def __init__(self):
        super().__init__('checkpoints_random')
        
        self.navigator = BasicNavigator()
        self.waypoints = []
         
        while 1:
            self.goal_pose = self.create_pose_stamped(random.uniform(-3.0, 3.0),random.uniform(-1.25, 1.25) , random.uniform(0.0, 6.28))
            self.waypoints.append(self.goal_pose)
            self.go_to_waypoint()
            if self.getKey() == 's':
            	print(self.waypoints)
            	self.follow_waypoints(self.waypoints)
            	break
            else:
                continue

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

    def go_to_waypoint(self):
        self.get_logger().info("Navigating to goal 1...")
        self.navigator.goToPose(self.goal_pose)
        self.check_task_complete()
        
    def follow_waypoints(self, waypoints):
        self.get_logger().info("going through poses...")
        self.navigator.goThroughPoses(waypoints)
        self.check_task_complete()
 
    def check_task_complete(self):
        """Check if the navigation task is complete"""
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(feedback)
        result = self.navigator.getResult()	
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

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
