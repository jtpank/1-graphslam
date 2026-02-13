#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <utility>
#include <Eigen/Geometry>
#include <Eigen/Core>


/*
Lot of frames here that we need to make sure we keep track of. 
Body: defined by the actual vehicle, (x, y) along the axis of the robot (x front, y left)
*/

using namespace std;
using namespace Eigen;

namespace utilities{

struct VehicleState {
    Vector3d pose;  // pose will always be in the world frame(x, y, theta)
    Vector2d vel;   // velocity in the world frame

    float omega;    // angular velocity in the body frame, positive ccw
    float a;        // linear acceleration in the body frame, position forward
    float alpha;    // angular acceleration in the body frame, positive ccw

    Quaterniond camera2body; // rotation from the camera to body frame. Going to need this as we start to do VO
    Vector3d cam2bodyt;      // translation from the camera to body frame

    Vector2d transformB2W( Vector2d& local_point)  {
        float cos_theta = std::cos(pose(2));
        float sin_theta = std::sin(pose(2));
        return Vector2d(
            pose(0) + local_point(0) * cos_theta - local_point(1) * sin_theta,
            pose(1) + local_point(0) * sin_theta + local_point(1) * cos_theta
        );
    }

    Vector2d transformW2B( Vector2d& world_point)  {
        float cos_theta = std::cos(pose(2));
        float sin_theta = std::sin(pose(2));
        return Vector2d(
            cos_theta * (world_point(0) - pose(0)) + sin_theta * (world_point(1) - pose(1)),
            -sin_theta * (world_point(0) - pose(0)) + cos_theta * (world_point(1) - pose(1))
        );
    }

    // Can add more state variables as needed,
    // so far just including position, velocity, acceleration.
    VehicleState() : pose(Vector3d::Zero()), omega(0.0f), a(0.0f), alpha(0.0f), camera2body(Quaterniond::Identity()), cam2bodyt(Vector3d::Zero()) {}
};

} // namespace utilities