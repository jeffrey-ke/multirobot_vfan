from ClusterController import ClusterController
import time
cc = ClusterController()
cc.DeclareRobots(["a", "b", "c"], [[0, 0, 0], [5, 0, 0], [2.5, 5, 0]])
start = time.time()
for i in range(1):
    cc.CalculateRDot([0, 0, 0, 0, 0, 0, 0, 0, 0])
end = time.time()
print(end - start)