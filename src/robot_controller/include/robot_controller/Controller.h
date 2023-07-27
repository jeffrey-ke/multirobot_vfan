#pragma once
#include <cmath>

typedef struct Pose {
    double x;
    double y;
    double theta;
} Pose;

typedef struct CmdVel {
    double angular_speed;
    double speed;
} CmdVel;

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

    Pose GetPose();
    Pose GetWp();
    CmdVel GetCmdVel();
};


