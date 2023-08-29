#pragma once
#include <cmath>
#define MIN_DIS_ERR 0.1
#define MIN_HEAD_ERR 0.001

typedef struct Pose {
    double x;
    double y;
    double theta;
} Pose;

typedef struct CmdVel {
    double angular_speed;
    double speed;
} CmdVel;

typedef struct Quaternion {
    double w, x, y, z;
} Quaternion;

typedef struct Euler {
    double roll, pitch, yaw;
} Euler;


class Controller
{
private:
    Pose pose_;
    Pose wp_;
    CmdVel cmdvel_;

    double k_p_angular_;
    double k_p_distance_;

public:
    Controller(double k_p_theta, double k_p_distance);
    ~Controller();
    void UpdateCmdVel();
    void UpdatePose(double x, double y, double theta);
    void UpdateWp(double x, double y);
    double CalculateAngularError();
    double CalculateDistanceError();
    Quaternion Normalize(const Quaternion& q);
    Euler QuaternionToEuler(const Quaternion& q_in);

    Pose GetPose();
    Pose GetWp();
    CmdVel GetCmdVel();
};


