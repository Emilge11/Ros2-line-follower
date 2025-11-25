class LineFollowerNode(Node):
    def __init__(self):
        # Initialize node
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
        self.left_sensor_sub = self.create_subscriber(Float64, '/e_puck/gs0', self.sensor_callback, 1)
        self.mid_sensor_sub = self.create_subscriber(Float64, '/e_puck/gs1', self.sensor_callback, 1)
        self.right_sensor_sub = self.create_subscriber(Float64, '/e_puck/gs2', self.sensor_callback, 1)
        # Create publisher
        # Initialize state variables
        # Create control timer

    def sensor_callback(self, msg):
        # Store sensor value when new data arrives
        pass

    def control_loop(self):
        # Get latest sensor readings
        # Apply line-following logic
        # Calculate desired velocities
        # Publish Twist message
        pass
