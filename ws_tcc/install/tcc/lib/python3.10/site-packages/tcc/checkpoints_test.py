import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import json

class Checkpoints(Node):

    def __init__(self):
        super().__init__('checkpoints')
        self.goal_handle = None
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.srv_start = self.create_service(Trigger, '/start', self.start_callback)
        self.srv_cancel = self.create_service(Trigger, '/cancel', self.cancel_callback)
        self.declare_parameter('checkpoints_file', '')
        
        # Publisher for visualizing waypoints in RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_marker', 10)
    
    def start_callback(self, request, response):
        try:
            checkpoints_file = self.get_parameter('checkpoints_file').get_parameter_value().string_value

            f = open(checkpoints_file)
            data = json.load(f)

            poses = []
            markers = MarkerArray()
            marker_id = 0
            
            for i in data['poses']:
                # Create PoseStamped message
                p = PoseStamped()
                p.header.frame_id = data['frame_id']
                p.pose.position.x = i['position']['x']
                p.pose.position.y = i['position']['y']
                p.pose.position.z = i['position']['z']
                p.pose.orientation.x = i['orientation']['x']
                p.pose.orientation.y = i['orientation']['y']
                p.pose.orientation.z = i['orientation']['z']
                p.pose.orientation.w = i['orientation']['w']
                poses.append(p)

                # Create Marker for RViz visualization
                marker = Marker()
                marker.header.frame_id = data['frame_id']
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "waypoints"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = p.pose.position
                marker.pose.orientation.w = 1.0  # Keep orientation neutral for visualization
                marker.scale.x = 0.2  # Set the scale of the sphere marker
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0  # Set alpha value for visibility
                marker.color.r = 1.0  # Set the color of the sphere
                marker.color.g = 0.0
                marker.color.b = 0.0
                markers.markers.append(marker)

                marker_id += 1
            
            # Publish the markers to RViz
            self.marker_pub.publish(markers)

            # Send waypoints to the action server
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = poses
            self._action_client.wait_for_server()
            self.goal_handle = self._action_client.send_goal_async(goal_msg)

            response.success = True
            response.message = "Success"
            return response
        except FileNotFoundError as e:
            self.get_logger().error(f"{e}")
            response.success = False
            response.message = f"{e}"
            return response

    def cancel_callback(self, request, response):
        if self.goal_handle:
            self.goal_handle.result().cancel_goal_async()

        response.success = True
        response.message = "Success"
        return response

def main(args=None):
    rclpy.init(args=args)

    action_client = Checkpoints()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

