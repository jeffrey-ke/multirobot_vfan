#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayDimension
from nav_core.Robot import Robot
from nav_core.Topics import robot0

class RobotNode(Node):

    def __init__(self):

        super().__init__("Node_for_robot")
        self.declare_parameter('pose', [0.0, 0.0])
        self.declare_parameter('id', "A")
        pose = self.get_parameter('pose').get_parameter_value().double_array_value
        id = self.get_parameter('id').get_parameter_value().string_value

        self.robot: Robot = Robot(id, pose)

        self.sensor_subscription = self.create_subscription(Float32, 
                                                     "/r" + id + "/sensor_readings", 
                                                     self.handle_sensor, 
                                                     10)

        self.cmdvel_subscription = self.create_subscription(Float32MultiArray,
                                                            "/r" + id + "/cmdvel",
                                                            self.handle_cmdvel,
                                                            10)
        
        self.pose_timer = self.create_timer(0.5, self.publish_pose)
        self.pose_publisher = self.create_publisher(Float32MultiArray, 
                                                    "/r" + id + "/pose", 
                                                    10)
        
        
    def publish_pose(self):
        pose = [float(x) for x in self.robot._pose]
        msg = Float32MultiArray()
        
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "length"
        msg.layout.dim[0].size = len(pose)
        msg.layout.dim[0].stride = len(pose)
        msg.layout.data_offset = 0

        msg.data = pose

        self.pose_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def handle_sensor(self, msg: Float32):
        self.get_logger().info('I heard: "%f" from topic: "%s"' % (msg.data, self.subscription.topic_name))
        self.robot.readSensor(msg.data)
    
    def handle_cmdvel(self, msg: Float32MultiArray):
        cmdvel = msg.data
        if len(cmdvel) != 2:
            raise Exception("Commanded velocity is incorrectly formatted.")
        self.robot.giveCommand(cmdvel)


def main(args=None):
    rclpy.init(args=args)
    
    R0 = RobotNode()
    rclpy.spin(R0)
    robot0.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    