from rclpy.node import Node
from geometry_msgs.msg import Point
import rclpy
import math

class PIDController:
    def __init__(self, kp, ki, kd, max_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.max_output = max_output

    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # Optional output clamping (smooth braking)
        if self.max_output is not None:
            output = max(min(output, self.max_output), -self.max_output)

        self.prev_error = error
        return output

class Robot(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.declare_parameters(
            namespace='', 
            parameters=[
                ('x', 400.0),
                ('y', 300.0),
                ('kp_x', 0.6),
                ('ki_x', 0.005),
                ('kd_x', 1.2),
                ('kp_y', 1),
                ('ki_y', 0.05),
                ('kd_y', 1.2),
                ('x_tolerance', 5),
                ('y_tolerance', 5),
                ('max_accel', 3),
                ('dt', 0.5),
                ("image_topic", 'field'),
            ]
        )
        self.init_params()
        self.pose_pub = self.create_publisher(Point, '/robot_pose', 10)
        self.goal_pose_pub = self.create_publisher(Point, '/goal_pose', 10)
        self.goal_pose_sub = self.create_subscription(Point, f'/{self.image_topic}_mouse_left', self.mouse_click_callback, 10)
        self.timer = self.create_timer(self.dt, self.pose_pub_callback)
        self.pid_controller_x = PIDController(
            kp=self.get_parameter('kp_x').value,
            ki=self.get_parameter('ki_x').value,
            kd=self.get_parameter('kd_x').value,
            max_output=3.0
        )
        self.pid_controller_y = PIDController(
            kp=self.get_parameter('kp_y').value,
            ki=self.get_parameter('ki_y').value,
            kd=self.get_parameter('kd_y').value,
            max_output=3.0
        )
        
    
    def init_params(self):
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.x_tolerance = self.get_parameter('x_tolerance').value
        self.y_tolerance = self.get_parameter('y_tolerance').value
        self.max_accel = self.get_parameter('max_accel').value
        self.dt = self.get_parameter('dt').value
        self.goal_x: int = self.x
        self.goal_y: int = self.y
        self.velocity_x = 0
        self.velocity_y = 0
        self.image_topic = self.get_parameter('image_topic').value

    def pose_pub_callback(self):
        msg = Point()

        accel_x = self.pid_controller_x.compute(self.goal_x, self.x, self.dt)
        self.velocity_x += accel_x * self.dt
        self.x += self.velocity_x * self.dt
        msg.x = self.x

        accel_y = self.pid_controller_y.compute(self.goal_y, self.y, self.dt)
        self.velocity_y += accel_y * self.dt
        self.y += self.velocity_y * self.dt
        msg.y = self.y

        # Condition to stop moving robot
        if abs(self.x - self.goal_x) <= self.x_tolerance and abs(self.y - self.goal_y) <= self.y_tolerance:
            self.velocity_x = 0
            self.velocity_y = 0
            self.x = self.goal_x
            self.y = self.goal_y
        
        self.pose_pub.publish(msg)
        # self.get_logger().info(f'x - {self.x - self.goal_x}, y - {self.y - self.goal_y}')

    def mouse_click_callback(self, msg: Point):
        self.goal_x = msg.y
        self.goal_y = msg.x

def main():
    rclpy.init()
    node = Robot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

