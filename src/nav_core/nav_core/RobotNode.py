#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayDimension
from nav_core.Robot import Robot
from nav_core.Topics import robot0

class RobotNode(Node):

    def __init__(self, id, sensor_topic, pose_topic, cmdvel_topic, pose):
        super().__init__("Node_for_robot" + id)
        
        self.robot: Robot = Robot(id, pose)

        self.sensor_subscription = self.create_subscription(Float32, 
                                                     sensor_topic, 
                                                     self.handle_sensor, 
                                                     10)

        self.cmdvel_subscription = self.create_subscription(Float32MultiArray,
                                                            cmdvel_topic,
                                                            self.handle_cmdvel,
                                                            10)
        
        self.pose_timer = self.create_timer(0.5, self.publish_pose)
        self.pose_publisher = self.create_publisher(Float32MultiArray, 
                                                    pose_topic, 
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
    
    R0 = RobotNode(robot0["id"], robot0["sensor_topic"], robot0["pose_topic"], robot0["cmdvel_topic"], robot0["init_pose"])
    rclpy.spin(R0)
    robot0.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    