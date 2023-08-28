from sympy import *
import numpy as np

"""
    Assumes 3 robots.
"""
class ClusterController:

    """
        Data structures to hold robot poses and r_dot
    """
    robots_pose_dict_ = {}
    r_dot_ = {}

    """
        symbolic vars for calculating jacobian
    """
    x1_, x2_, x3_, y1_, y2_, y3_, theta_1_, theta_2_, theta_3_ = symbols("x1 x2 x3 y1 y2 y3 th1 th2 th3")

    """
        Forward kinematic transform equations that use sympy's
        symbolic equations data structure.
    """
    x_c_ = (x1_ + x2_ + x3_) / 3.0
    y_c_ = (y1_ + y2_ + y3_) / 3.0
    theta_c_ = atan2( (2.0/3.0) * x1_ - (1.0/3.0) * (x2_ + x3_), 
                     (2.0/3.0) * y1_ - (1.0/3.0) * (y2_ + y3_))
    phi_1_ = theta_1_ + theta_c_
    phi_2_ = theta_2_ + theta_c_
    phi_3_ = theta_3_ + theta_c_
    p_ = sqrt( Pow(x1_ - x2_, 2) + Pow(y1_ - y2_, 2))
    q_ = sqrt( Pow(x3_ - x1_, 2) + Pow(y3_ - y1_, 2))
    beta_ = acos( (Pow(p_, 2) + Pow(q_, 2) - Pow(x3_ - x2_, 2) - Pow(y3_ - y2_, 2)) / 
                 (2 * p_ * q_) )
    
    """
        Matrix form of the kinematic transform
    """
    KIN_ = Matrix([x_c_, y_c_, theta_c_, phi_1_, phi_2_, phi_3_, p_, q_, beta_])
    R_ = Matrix([x1_, y1_, theta_1_, x2_, y2_, theta_2_, x3_, y3_, theta_3_])
    
    """
        Jacobian is a function that evaluates and returns J(R) as a numpy matrix. Todo: stop building the lambda function
        on every function call.
    """
    def Jacobian(self, x1_, y1_, theta_1_, x2_, y2_, theta_2_, x3_, y3_, theta_3_):
        jacob = lambdify([self.x1_, self.y1_, self.theta_1_, self.x2_, self.y2_, self.theta_2_, self.x3_, self.y3_, self.theta_3_], 
                            self.KIN_.jacobian(self.R_), modules="numpy")
        return jacob(x1_, y1_, theta_1_, x2_, y2_, theta_2_, x3_, y3_, theta_3_)

    def __init__(self):
        pass

    def DeclareRobots(self, ids, poses):
        if len(ids) != 3 or len(poses) != 3:
            raise Exception("Please provide 3 robots and their poses")
        for id, pose in zip(ids, poses):
            if len(pose) != 3:
                raise Exception("Pose must be [x, y, theta]")
            self.robots_pose_dict_[id] = pose
    
    def UpdatePose(self, id, pose):
        if id not in list(self.robots_pose_dict_.keys()):
            raise Exception("No robot has that id.")
        self.robots_pose_dict_[id] = pose

    def UpdateAllPoses(self, ids, poses):
        for id, pose in zip(ids, poses):
            self.UpdatePose(id, pose)
    
    def CalculateRDot(self, c_dot):
        if len(c_dot) != 9:
            raise Exception("Commanded cluster velocity MUST be 9 elements long. \
                            Read Kitts 3-robot cluster kinematic equations for \
                            what the cluster space variables are.")
        R = [element for pose in self.robots_pose_dict_.values() for element in pose]
        if len(R) != 9:
            print(len(R))
            raise Exception("Big problem. You don't have 3 correct poses of three variables \
                            x, y, and theta.")
        print("R: " + " ".join([str(r) for r in R]))
        J_with_current_values = self.Jacobian(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8])
        np.nan_to_num(J_with_current_values, posinf=0.0, neginf=0.0, copy=False)

        if (np.isclose(np.linalg.det(J_with_current_values), 0)):
            print("Singular matrix??")
            return np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        else:
            return np.linalg.inv(J_with_current_values) @ c_dot