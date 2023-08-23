
from cluster_controller.ClusterController import ClusterController
import numpy as np
def test_inv_jacob():
    c = ClusterController()
    c.DeclareRobots(["a", "b", "c"], [[0, 0, 0], [5, 0, 0], [2.5, 5, 0]])
    np.set_printoptions(suppress=True)
    for elem in np.equal(c.CalculateRDot([0, 0, 0, 0, 0, 0, 0, 0, 0]), [0, 0, 0, 0, 0, 0, 0, 0 , 0]):
        assert elem
    assert(False)
