import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.parameter_service import SetParametersResult

class ScanExample(Node):
    
    def __init__(self):
        super().__init__('scan_example')
        # parameter
        self.declare_parameter('obstacle_detect_range', 0.5)

        # parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.scan_subscription = self.create_subscription(
                    LaserScan,
                    '/scan',
                    self.listener_callback,
                    10
                )
        self.scan_subscription # prevent unused variable warning
        
        self.obstacle_detect_range_visualization_publisher = self.create_publisher(
                LaserScan,
                '/obstacle_detect_range_visualization',
                10
            )
        
        self.obstacle_detect_scan_publisher = self.create_publisher(
                LaserScan,
                '/obstacle_detect_scan',
                10
            )
        
    def parameter_callback(self, params):
        for param in params:
            if param.name in ['obstacle_detect_range']:
                self.get_logger().info(f"Parameter '{param.name}' changed to {param.value}")
        return SetParametersResult(successful=True)
        
    def listener_callback(self, msg):
        obstacle_detect_range = self.get_parameter('obstacle_detect_range').value

        obstacle_detected = False
          
        obstacle_detect_range_visualization_msg = LaserScan() 
        
        obstacle_detect_range_visualization_msg.header = msg.header
        obstacle_detect_range_visualization_msg.angle_min = msg.angle_min
        obstacle_detect_range_visualization_msg.angle_max = msg.angle_max
        obstacle_detect_range_visualization_msg.angle_increment = msg.angle_increment
        obstacle_detect_range_visualization_msg.time_increment = msg.time_increment
        obstacle_detect_range_visualization_msg.scan_time = msg.scan_time
        obstacle_detect_range_visualization_msg.range_min = msg.range_min
        obstacle_detect_range_visualization_msg.range_max = msg.range_max
        obstacle_detect_range_visualization_msg.ranges = [obstacle_detect_range for i in range(len(msg.ranges))]
        obstacle_detect_range_visualization_msg.intensities =  msg.intensities

        obstacle_detect_scan = LaserScan() 
        
        obstacle_detect_scan.header = msg.header
        obstacle_detect_scan.angle_min = msg.angle_min
        obstacle_detect_scan.angle_max = msg.angle_max
        obstacle_detect_scan.angle_increment = msg.angle_increment
        obstacle_detect_scan.time_increment = msg.time_increment
        obstacle_detect_scan.scan_time = msg.scan_time
        obstacle_detect_scan.range_min = msg.range_min
        obstacle_detect_scan.range_max = msg.range_max
        obstacle_detect_scan.ranges = [float('inf') for i in range(len(msg.ranges))]
        obstacle_detect_scan.intensities =  msg.intensities

        
        for x in range(len(obstacle_detect_range_visualization_msg.ranges)):
            if msg.ranges[x] < obstacle_detect_range:
                obstacle_detect_scan.ranges[x] = msg.ranges[x]
                if not obstacle_detected:
                    self.get_logger().info(f"obstacle detect")
                    obstacle_detected = True


        self.obstacle_detect_range_visualization_publisher.publish(obstacle_detect_range_visualization_msg)
        self.obstacle_detect_scan_publisher.publish(obstacle_detect_scan)


def main(args=None):
    rclpy.init(args=args)

    scan_example = ScanExample()

    rclpy.spin(scan_example)
    scan_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()