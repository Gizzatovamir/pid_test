from rclpy.node import Node
from geometry_msgs.msg import Point
import rclpy

class Robot(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.declare_parameters(
            namespace='', 
            parameters=[
                ('x', 400.0),
                ('y', 300.0),
                ('vx', 0.0),
                ('vy', 0.0),
                ('ax', 0.0),
                ('ay', 0.0),
                ('g_x', 400.0),
                ('g_y', 300.0),
            ]
        )
        self.init_params()
        self.pose_pub = self.create_publisher(Point, '/robot_pose', 10)
        # self.goal_pose_pub = self.create_publisher(Point, '/goal_pose', 10)
        self.timer = self.create_timer(0.5, self.pose_pub_callback)
        
    
    def init_params(self):
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.g_x = self.get_parameter('g_x').value
        self.g_y = self.get_parameter('g_y').value

    def pose_pub_callback(self):
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        self.pose_pub.publish(msg)
        self.get_logger().info(f'msg - {msg.x, msg.y}')
        self.x += 1
        self.y += 1
def main():
    rclpy.init()
    node = Robot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

