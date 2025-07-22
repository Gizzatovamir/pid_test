from rclpy.node import Node
from geometry_msgs.msg import Point
import rclpy

class Pid(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.declare_parameters(
            namespace='', 
            parameters=[
                ('vx', 0.0),
                ('vy', 0.0),
                ('ax', 0.0),
                ('ay', 0.0),
                ('kp_x', 0.0),
                ('ki_x', 0.0),
                ('kd_x', 0.0),
                ('kp_y', 0.0),
                ('ki_y', 0.0),
                ('kd_y', 0.0),
            ]
        )
        self.init_params()
        self.pose_pub = self.create_subscription(Point, '/robot_pose', self.pose_callback, 10)
        self.accel_pub = self.create_publisher(Point, '/robot_accel', 10)
        self.timer = self.create_timer(0.5, self.pose_pub_callback)
        self.prev_error = 0
        self.integral = 0
    
    
    def init_params(self):
        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.ax = self.get_parameter('ax').value
        self.ay = self.get_parameter('ay').value
        self.kp_x = self.get_parameter('kp_x').value
        self.ki_x = self.get_parameter('ki_x').value
        self.kd_x = self.get_parameter('kd_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.ki_y = self.get_parameter('ki_y').value
        self.kd_y = self.get_parameter('kd_y').value
    
    def update(self, target_x, measured_x, target_ty, measured_ty, dt):
        # --- X Axis ---
        error_x = target_x - measured_x
        self.integral_x += error_x * dt
        derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0

        output_x = (
            self.kp_x * error_x +
            self.ki_x * self.integral_x +
            self.kd_x * derivative_x
        )
        self.prev_error_x = error_x

        # --- TY Axis ---
        error_ty = target_ty - measured_ty
        self.integral_ty += error_ty * dt
        derivative_ty = (error_ty - self.prev_error_ty) / dt if dt > 0 else 0

        output_ty = (
            self.kp_ty * error_ty +
            self.ki_ty * self.integral_ty +
            self.kd_ty * derivative_ty
        )
        self.prev_error_ty = error_ty

        return output_x, output_ty

    def pose_callback(self, msg: Point):
        new_accel_x, new_accel_y = self.update()