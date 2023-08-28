
from cluster_controller.ClusterController import ClusterController
import numpy as np
def test_inv_jacob():
    c = ClusterController()
    c.DeclareRobots(["a", "b", "c"], [[0, 0, 0], [5, 0, 0], [2.5, 5, 0]])
    np.set_printoptions(suppress=True)
    for elem in np.equal(c.CalculateRDot([0, 0, 0, 0, 0, 0, 0, 0, 0]), [0, 0, 0, 0, 0, 0, 0, 0 , 0]):
        assert elem

def test_inv_jacob2():
    c = ClusterController()
    c.DeclareRobots(["a", "b", "c"], [[0, 0, 0], [5, 0, 0], [2.5, 5, 0]])
    np.set_printoptions(suppress=True)
    r_dot = c.CalculateRDot([10.0, 10.0, 1.0, 0, 0, 0, 0, 0, 0])
    print(r_dot)
    for elem in np.equal(r_dot, [0, 0, 0, 0, 0, 0, 0, 0 , 0]):
        assert not elem