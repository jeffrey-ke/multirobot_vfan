import rclpy
from rclpy.node import Node
from sim_field.simulated_field import SimulatedField
from std_msgs.msg import String, Float32MultiArray

class SimFieldNode(Node):


    def __init__(self):
        super().__init__("sim")
        self.declare_parameter("r0_pose_topic", "/x")
        self.declare_parameter("r1_pose_topic", "/x")  
        self.declare_parameter("r2_pose_topic", "/x")  

        robot_pose_topics = [self.get_parameter(field).get_parameter_value().string_value for field in ["r0_pose_topic", "r1_pose_topic", "r2_pose_topic"]]

        self.create_subscription(Float32MultiArray, robot_pose_topics[0], self.updateR0Pose, 10)
        self.create_subscription(Float32MultiArray, robot_pose_topics[1], self.updateR1Pose, 10)
        self.create_subscription(Float32MultiArray, robot_pose_topics[2], self.updateR2Pose, 10)

        self.sim_field_ = SimulatedField()
        self.sim_field_.spawnFeature(location=(50,50))
        self.sim_field_.spawnRobot("0", (0, 0))
        self.sim_field_.spawnRobot("1", (0, 0))
        self.sim_field_.spawnRobot("2", (0, 0))

    def updateR0Pose(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            raise Exception("Published pose is not in format of (x, y).")
        self.sim_field_.moveRobot("0", msg.data)
    
    def updateR1Pose(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            raise Exception("Published pose is not in format of (x, y).")
        self.sim_field_.moveRobot("1", msg.data)

    def updateR2Pose(self, msg:Float32MultiArray):
        if len(msg.data) != 2:
            raise Exception("Published pose is not in format of (x, y).")
        self.sim_field_.moveRobot("2", msg.data)

def main(args=None):
    rclpy.init(args=args)
    simfield = SimFieldNode()
    rclpy.spin(simfield)
    simfield.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()