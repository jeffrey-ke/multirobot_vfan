#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, MultiArrayDimension
import numpy as np
class ClusterNode(Node):
    _r0_sensor_reading = -99.0
    _r1_sensor_reading = -99.0
    _r2_sensor_reading = -99.0

    _r0_pose = [0.0, 0.0]
    _r1_pose = [0.0, 0.0]
    _r2_pose = [0.0, 0.0]

    _c_vel = [1.0, 1.0]

    def __init__(self):
        super().__init__("ClusterNode")
        """
            Declaring launch parameters.
        """
        self.declare_parameter('robot0_id', "0")
        self.declare_parameter('robot1_id', "1")
        self.declare_parameter('robot2_id', "2")
        ids = [self.get_parameter(field).get_parameter_value().string_value for field in ["robot0_id", "robot1_id", "robot2_id"]]


        """
            Subscriptions for each of the robot's sensor readings
        """
        self.create_subscription(Float32MultiArray, 
                                 "/r" + ids[0] + "/sensor_readings", 
                                 self.handle_r0sensor, 
                                 10)
        self.create_subscription(Float32MultiArray,
                                 "/r" + ids[1] + "/sensor_readings",
                                 self.handle_r1sensor,
                                 10)
        self.create_subscription(Float32MultiArray,
                                 "/r" + ids[2] + "/sensor_readings",
                                 self.handle_r2sensor,
                                 10)
        
        """
            Subscriptions for each of the robot's poses.
        """
        self.create_subscription(Float32MultiArray,
                                 "/r" + ids[0] + "/pose",
                                 self.handle_r0pose,
                                 10)
        self.create_subscription(Float32MultiArray,
                                 "/r" + ids[1] + "/pose",
                                 self.handle_r1pose,
                                 10)
        self.create_subscription(Float32MultiArray,
                                 "/r" + ids[2] + "/pose",
                                 self.handle_r2pose,
                                 10)

        """
            Publishers for each of the commanded velocities for each of the robots.
        """
        self._r_timer = self.create_timer(0.5, self.publish_r_cmdvel)
        self._robot0cmdvel_publisher = self.create_publisher(Float32MultiArray,
                                                            "/r" + ids[0] + "/cmdvel",
                                                            10)
        
        self._robot1cmdvel_publisher = self.create_publisher(Float32MultiArray,
                                                            "/r" + ids[1] + "/cmdvel",
                                                            10)
        
        self._robot2cmdvel_publisher = self.create_publisher(Float32MultiArray,
                                                            "/r" + ids[2] + "/cmdvel",
                                                            10)
        
    def publish_r_cmdvel(self):
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "length"
        msg.layout.dim[0].size = len(self._c_vel)
        msg.layout.dim[0].stride = len(self._c_vel)
        msg.layout.data_offset = 0
        msg.data = [float(x) for x in self._c_vel]
        self._robot0cmdvel_publisher.publish(msg)
        self._robot1cmdvel_publisher.publish(msg)
        self._robot2cmdvel_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.calculate_c_vel()

    def handle_r0sensor(self, msg: Float32MultiArray):
        self._r0_sensor_reading = msg.data[0]
        
    def handle_r1sensor(self, msg: Float32MultiArray):
        self._r1_sensor_reading = msg.data[0]
    
    def handle_r2sensor(self, msg: Float32MultiArray):
        self._r2_sensor_reading = msg.data[0]

    def handle_r0pose(self, msg: Float32MultiArray):
        if len(msg.data) != len(self._r0_pose):
            raise Exception("Cluster node does not have the same pose vector length as what\'s being published.")
        self._r0_pose = msg.data
    
    def handle_r1pose(self, msg: Float32MultiArray):
        if len(msg.data) != len(self._r1_pose):
            raise Exception("Cluster node does not have the same pose vector length as what\'s being published.")
        self._r1_pose = msg.data


    def handle_r2pose(self, msg: Float32MultiArray):
        if len(msg.data) != len(self._r1_pose):
            raise Exception("Cluster node does not have the same pose vector length as what\'s being published.")
        self._r2_pose = msg.data

    def calculate_c_vel(self):
        r0 = np.array([self._r0_pose[0], self._r0_pose[1], self._r0_sensor_reading])
        r1 = np.array([self._r1_pose[0], self._r1_pose[1], self._r1_sensor_reading])
        r2 = np.array([self._r2_pose[0], self._r2_pose[1], self._r2_sensor_reading])

        r01 = r1 - r0
        r02 = r2 - r0

        self._c_vel = -np.cross(r01, r02)[:2]
        pass


def main(args=None):
    rclpy.init(args=args)

    cl = ClusterNode()
    rclpy.spin(cl)
    cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()