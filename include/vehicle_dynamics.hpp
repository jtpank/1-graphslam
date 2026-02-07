#pragma once

#include <cmath>
#include <vector>
#include <utility>
#include "Utilities.hpp"

// Keeping this namespace so that if there are different dynamics models we can easily swap them out and maintain modularity.
namespace vehicle_dynamics {

struct DriveParams {
    float track_width;
    float wheel_radius;

    float mass;
    float inertia;
    float max_velocity;
    float max_omega;        //

    DriveParams()
        :track_width(0.5f),
        wheel_radius(0.5f),
        mass(1.0),
        inertia(1.0),
        max_velocity(1.0),
        max_omega(1.0f) {}

};

class VehicleDynamics{
    public:
        VehicleDynamics(const DriveParams& params);
        ~VehicleDynamics();

        DifferentialDriveControl tanktoDiff(DriveControlOutput tank_control);
        DriveControlOutput difftoTank(DifferentialDriveControl diff_control);

        VehicleState predict(VehicleState current_state, DifferentialDriveControl control, float dt);
        Pose2D predictPose(Pose2D current_pose, DifferentialDriveControl control, float dt);

        private:
            DriveParams params_;
            Pose2D integratePose(const Pose2D& pose, float v, float omega, float dt) const;

};


} // namespace vehicle_dynamics