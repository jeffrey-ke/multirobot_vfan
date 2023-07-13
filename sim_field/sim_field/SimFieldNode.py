import rclpy
from rclpy.node import Node
from sim_field.simulated_field import SimulatedField
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension

class SimFieldNode(Node):


    def __init__(self):
        super().__init__("sim")
        """
            ROS parameters
        """
        self.declare_parameter("r0_pose_topic", "/x")
        self.declare_parameter("r1_pose_topic", "/x")  
        self.declare_parameter("r2_pose_topic", "/x")
        robot_pose_topics = [self.get_parameter(field).get_parameter_value().string_value for field in ["r0_pose_topic", "r1_pose_topic", "r2_pose_topic"]]

        self.declare_parameter("r0_sensor_topic", "/x")
        self.declare_parameter("r1_sensor_topic", "/x")  
        self.declare_parameter("r2_sensor_topic", "/x")  
        robot_sensor_topics = [self.get_parameter(field).get_parameter_value().string_value for field in ["r0_sensor_topic", "r1_sensor_topic", "r2_sensor_topic"]]

        """
            Subscriptions
        """
        self.create_subscription(Float32MultiArray, robot_pose_topics[0], self.updateR0Pose, 10)
        self.create_subscription(Float32MultiArray, robot_pose_topics[1], self.updateR1Pose, 10)
        self.create_subscription(Float32MultiArray, robot_pose_topics[2], self.updateR2Pose, 10)


        """
            Publishers
        """
        self._sensor_timer = self.create_timer(0.5, self.publishSensorReadings)
        self._r0_sensor_publisher = self.create_publisher(Float32MultiArray,
                                                          robot_sensor_topics[0],
                                                          10)
        self._r1_sensor_publisher = self.create_publisher(Float32MultiArray,
                                                          robot_sensor_topics[1],
                                                          10)
        self._r2_sensor_publisher = self.create_publisher(Float32MultiArray,
                                                          robot_sensor_topics[2],
                                                          10)
        

        """
            Timer to periodically update the simfield
        """
        self._field_update_timer = self.create_timer(0.5, self.updateField)


        """
            private member variables
        """
        self._sim_field = SimulatedField(feature_sigma=40)
        self._sim_field.spawnFeature(location=(50,50), sigma=40)
        self._sim_field.spawnRobot("0", (0, 0))
        self._sim_field.spawnRobot("1", (0, 0))
        self._sim_field.spawnRobot("2", (0, 0))

        self.r0pose = [0,0]
        self.r1pose = [0,0]
        self.r2pose = [0,0]

    def publishSensorReadings(self):
        r0_reading = [ float(self._sim_field._robot_state["0"]["sensor_reading"]) ]
        r1_reading = [ float(self._sim_field._robot_state["1"]["sensor_reading"]) ]
        r2_reading = [ float(self._sim_field._robot_state["2"]["sensor_reading"]) ]

        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "length"
        msg.layout.dim[0].size = len(r0_reading)
        msg.layout.dim[0].stride = len(r0_reading)
        msg.layout.data_offset = 0
        msg.data = r0_reading
        self._r0_sensor_publisher.publish(msg)

        msg.data = r1_reading
        self._r1_sensor_publisher.publish(msg)

        msg.data = r2_reading
        self._r2_sensor_publisher.publish(msg)

    def updateField(self):
        self.get_logger().info("Poses: %s" % [self.r0pose, self.r1pose, self.r2pose])
        self._sim_field.moveRobot("0", self.r0pose)
        self._sim_field.moveRobot("1", self.r1pose)
        self._sim_field.moveRobot("2", self.r2pose)

    def updateR0Pose(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            raise Exception("Published pose is not in format of (x, y).")
        self.r0pose = msg.data
    
    def updateR1Pose(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            raise Exception("Published pose is not in format of (x, y).")
        self.r1pose = msg.data

    def updateR2Pose(self, msg:Float32MultiArray):
        if len(msg.data) != 2:
            raise Exception("Published pose is not in format of (x, y).")
        self.r2pose = msg.data 

def main(args=None):
    rclpy.init(args=args)
    simfield = SimFieldNode()
    rclpy.spin(simfield)
    simfield.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()