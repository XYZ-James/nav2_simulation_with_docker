import rclpy
from rclpy.node import Node
from rclpy.parameter_service import SetParametersResult
from geometry_msgs.msg import Twist

class CmdVelExample(Node):
    
    def __init__(self):
        super().__init__('cmd_vel_example')
        
        # parameter
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 2.0)
        
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.cmd_subscription # prevent unused variable warning
        
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/new_cmd_vel',
            10
        )
        
        # parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
    def listener_callback(self, msg):
        linear_scale = self.get_parameter('linear_scale').value
        angular_scale = self.get_parameter('angular_scale').value
    
        new_msg = Twist()
        
        new_msg.linear.x = msg.linear.x * linear_scale
        new_msg.linear.y = msg.linear.y * linear_scale
        new_msg.linear.z = msg.linear.z * linear_scale
        
        new_msg.angular.x = msg.angular.x * angular_scale
        new_msg.angular.y = msg.angular.y * angular_scale
        new_msg.angular.z = msg.angular.z * angular_scale
        
        self.cmd_publisher.publish(new_msg)
        self.get_logger().info(f"---")
        self.get_logger().info(f"Publishing with linear_scale={linear_scale}, angular_scale={angular_scale}")
        self.get_logger().info(f"raw_linear  -> scaled_linear  : {msg.linear.x} -> {new_msg.linear.x}")
        self.get_logger().info(f"raw_angular -> scaled_angular : {msg.angular.z} -> {new_msg.angular.z}")
        self.get_logger().info(f"---")
        
    def parameter_callback(self, params):
        for param in params:
            if param.name in ['linear_scale', 'angular_scale']:
                self.get_logger().info(f"Parameter '{param.name}' changed to {param.value}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_example = CmdVelExample()

    rclpy.spin(cmd_vel_example)
    cmd_vel_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()