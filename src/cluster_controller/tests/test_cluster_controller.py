import sys
sys.path.append("/home/jeffrey/repo/ros_ws/src/cluster_controller/cluster_controller")

from cluster_controller.cluster_controller import ClusterController
def test_inv_jacob():
    c = ClusterController()
    c.DeclareRobots(["a", "b", "c"], [[0, 0, 0], [5, 0, 0], [2.5, 5, 0]])
    assert c.CalculateRDot([0, 0, 0, 0, 0, 0, 0, 0, 0]) \
                                == [0, 0, 0, 0, 0, 0, 0, 0 , 0]
