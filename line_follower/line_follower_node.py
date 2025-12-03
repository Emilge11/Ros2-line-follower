import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from functools import partial

class LineFollowerNode(Node):
    def __init__(self):
        # Initialize node
        self.left_val = 0.0
        self.mid_val = 0.0
        self.right_val = 0.0
        super().__init__('line_follower_node')
        # Declare parameters
        self.declare_parameter('MAX_SPEED', 0.0)
        self.declare_parameter('TURN_LEFT_SPEED', 0.0)
        self.declare_parameter('TURN_RIGHT_SPEED', 0.0)
        self.declare_parameter('SENSOR_THRESHOLD', 0.0)
        
        self.MAX_SPEED = self.get_parameter('MAX_SPEED').value
        self.TURN_LEFT_SPEED = self.get_parameter('TURN_LEFT_SPEED').value
        self.TURN_RIGHT_SPEED = self.get_parameter('TURN_RIGHT_SPEED').value
        self.SENSOR_THRESHOLD = self.get_parameter('SENSOR_THRESHOLD').value
        # Create subscribers
        self.left_sensor_sub = self.create_subscription(
        Float64, '/e_puck/gs0', partial(self.sensor_callback, name='left'), 1)
        self.mid_sensor_sub = self.create_subscription(
        Float64, '/e_puck/gs1', partial(self.sensor_callback, name='mid'), 1)
        self.right_sensor_sub = self.create_subscription(
        Float64, '/e_puck/gs2', partial(self.sensor_callback, name='right'), 1)
        # Create publisher
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        # Initialize state variables
        self.current_state = 'forward'
        self.counter = 0
        # Create control timer
        self.timer = self.create_timer(0.05, self.control_loop)

    def sensor_callback(self, msg, name):
        # Store sensor value when new data arrives
        if name == 'left':
            self.left_val = msg.data
        elif name == 'mid':
            self.mid_val = msg.data
        elif name == 'right':
            self.right_val = msg.data
            
    def control_loop(self):
        # Get latest sensor readings
        left = self.left_val
        mid = self.mid_val
        right = self.right_val
        
        vel_cmd = Twist()
        # Apply line-following logic
        if left > self.SENSOR_THRESHOLD and right < self.SENSOR_THRESHOLD:
            self.current_state = 'turn_left'
            self.counter = 0
        elif right > self.SENSOR_THRESHOLD and left < self.SENSOR_THRESHOLD:
            self.current_state = 'turn_right'
            self.counter = 0
        else:
            self.current_state = 'forward'
        # Calculate desired velocities
        if self.current_state == 'forward':
            vel_cmd.linear.x = self.MAX_SPEED
            vel_cmd.angular.z = 0.0
        
        elif self.current_state == 'turn_right':
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = -self.TURN_RIGHT_SPEED * self.MAX_SPEED
            
            if self.counter >= 5:
                self.current_state = 'forward'
                
        elif self.current_state == 'turn_left':
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = self.TURN_LEFT_SPEED * self.MAX_SPEED
                
            if self.counter >= 5:
                self.current_state = 'forward'
        # Publish Twist message
        self.vel_pub.publish(vel_cmd)
        
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
