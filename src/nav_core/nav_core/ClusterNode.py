#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, MultiArrayDimension
from nav_core.Topics import robot0

class ClusterNode(Node):
    r0sensorReading = -99
    def __init__(self):
        super().__init__("ClusterNode")
        self.declare_parameter('robot0_id', "0")
        self.declare_parameter('robot1_id', "1")
        self.declare_parameter('robot2_id', "2")
        ids = [self.get_parameter(field).get_parameter_value().string_value for field in ["robot0_id", "robot1_id", "robot2_id"]]

        self.robot0sensor_subscription = self.create_subscription(Float32,
                                                                  "/r" + ids[0] + "/sensor_readings",
                                                                  self.handle_r0sensor,
                                                                  10)
        self.r0_timer = self.create_timer(0.5, self.publish_r0cmdvel)
        self.robot0cmdvel_publisher = self.create_publisher(Float32MultiArray,
                                                            "/r" + ids[0] + "/cmdvel",
                                                            10)
        
    def publish_r0cmdvel(self):
        cmdvel = [1.0, 1.0]
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "length"
        msg.layout.dim[0].size = len(cmdvel)
        msg.layout.dim[0].stride = len(cmdvel)
        msg.layout.data_offset = 0
        msg.data = cmdvel
        self.robot0cmdvel_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    def handle_r0sensor(self, msg:Float32):
        self.r0sensorReading = msg.data
        pass

def main(args=None):
    rclpy.init(args=args)

    cl = ClusterNode()
    rclpy.spin(cl)
    cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()