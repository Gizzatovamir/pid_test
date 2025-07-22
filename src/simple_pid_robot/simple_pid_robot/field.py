from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Point
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class Field(Node):
    def __init__(self):
        super().__init__('field_node')
        self.declare_parameters(namespace='', parameters=[
            ('width', 800),
            ('height', 600),
            ('robot_radius', 10),
            ('goal_radius', 5),
        ]
        )
        self.init_params()
        self.robot_pose_sub = self.create_subscription(Point, '/robot_pose', self.pose_callback, 10)
        # self.goal_pose_sub = self.create_subscription(Point, '/goal_pose', self.pose_callback, 10)
        self.field: np.ndarray = np.zeros(
            (
                self.height,
                self.width,
                3
            ),
            dtype=np.uint8
        )
        self.field_publisher = self.create_publisher(Image, '/field', 10)
        self.bridge = CvBridge()
        self.robot_pose: Point = Point()
        self.goal_x: int = 100
        self.goal_y: int = 200
    
    def init_params(self):
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value        
        self.robot_radius = self.get_parameter('robot_radius').value   
        self.robot_color = np.array([255,0,0])
        self.goal_color = np.array([0, 255, 0])
        self.goal_radius = self.get_parameter('goal_radius').value   

    def draw_circle(self, image, x, y, r, color): 
        xx, yy = np.mgrid[:self.height, :self.width]
        circle = (xx - x) ** 2 + (yy - y) ** 2 <= r*2

        image[circle] = color

    def draw(self, robot_pose: Point):
        res_img = self.field.copy()
        
        self.draw_circle(
            image=res_img, 
            x=robot_pose.x, 
            y=robot_pose.y, 
            r=self.robot_radius, 
            color=self.robot_color
        )

        self.draw_circle(
            image=res_img, 
            x=self.goal_x, 
            y=self.goal_y, 
            r=self.goal_radius, 
            color=self.goal_color
        )

        return res_img


    def pose_callback(self, msg: Point):
        self.get_logger().info(f'Got new pose for robot - {msg.x, msg.y}')
        image = self.draw(msg)
        ros_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.field_publisher.publish(ros_image_msg)

def main():
    rclpy.init()
    node = Field()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
