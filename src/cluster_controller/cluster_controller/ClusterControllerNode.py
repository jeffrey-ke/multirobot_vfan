import rclpy
from rclpy.node import Node
from .ClusterController import ClusterController
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from custom_msgs.msg import Vel, Pose
from tf_transformations import euler_from_quaternion
import time
from yaml import load
class ClusterControllerNode(Node):

    cc_ = ClusterController()
    robot_pose_dict_ = {}
    topic_subs_ = {}
    topic_pubs_ = {}
    r_dot_ = range(9)
    c_dot_ = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    def __init__(self):
        super().__init__("ccnode")
        with open("/home/jeffrey/repo/ros_ws/src/cluster_controller/config/config.yaml", "r") as config_stream:
            config = load(config_stream)
        if len(config) != 3:
            raise Exception("You didn\'t provide 3 robots in the config")
        
        self.c_vel_sub_ = self.create_subscription(Vel, "/cluster_controller/c_vel", self.CVelCb, 10)
        for id in config.keys():
            wp_topic, pose_topic = config[id]["topics"]
            self.topic_subs_[pose_topic] = self.create_subscription(Pose, 
                                                                    pose_topic, 
                                                                    lambda msg, id=id: self.PoseCb(id, msg), 
                                                                    10)
            self.topic_pubs_[wp_topic] = self.create_publisher(Pose,
                                                                wp_topic,
                                                                10)
            
        self.timer_ = self.create_timer(0.5, self.Loop)
        ids = [id for id in config.keys()]
        poses = [config[id]["pose"] for id in config.keys()]
        self.cc_.DeclareRobots(ids, poses)
    
    def Loop(self):
        self.CalculateAndPublishRDot()
    
    def CVelCb(self, msg: Vel):
        self.c_dot_ = [msg.x_dot, msg.y_dot, msg.th_dot, 0, 0, 0, 0, 0, 0]
        self.get_logger().info("\n\nCC got: [" + " ".join([str(c) for c in self.c_dot_]))

    def PoseCb(self, id, msg):
        x, y = msg.position.x, msg.position.y #only care about x and y
        o = msg.orientation
        yaw = euler_from_quaternion([o.w, o.x, o.y, o.z])[-1]
        self.robot_pose_dict_[id] = [x, y, yaw]
        self.get_logger().info("Id: {}\n\tPose: {}".format(id, " ".join([str(e) for e in [x, y, yaw]])))
        self.cc_.UpdatePose(id, [x, y, yaw])
            
    
    def CalculateAndPublishRDot(self):
        self.get_logger().info("Calculating R_dot with C_dot: [" + " ".join([str(c) for c in self.c_dot_]))
        self.r_dot_ = self.cc_.CalculateRDot(self.c_dot_)

        self.get_logger().info("R_dot: [{}]".format(" ".join([str(r) for r in self.r_dot_])))
        r_dot_1, r_dot_2, r_dot_3 = self.r_dot_[0:3], self.r_dot_[3:6], self.r_dot_[6:9]
        wp_msg_1, wp_msg_2, wp_msg_3 = Pose(), Pose(), Pose()

        if len(self.topic_pubs_) != 3:
            raise Exception("Something wrong with config. 3 robots not spawned.")
        for topic, r_dot, wp_msg in zip(self.topic_pubs_, 
                                        [r_dot_1, r_dot_2, r_dot_3], 
                                        [wp_msg_1, wp_msg_2, wp_msg_3]):
            wp_msg.x = float(r_dot[0])
            wp_msg.y = float(r_dot[1])
            wp_msg.theta = float(r_dot[2])
            self.topic_pubs_[topic].publish(wp_msg)
            self.get_logger().info("\n\nPublishing waypoints..")
            self.get_logger().info("\tTopic: {}".format(topic))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ClusterControllerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()