#pragma once

#include <cmath>
#include <vector>
#include "Utilities.hpp"

using namespace utilities;

// Keeping this namespace so that if there are different dynamics models we can easily swap them out and maintain modularity.
namespace vehicle_dynamics {

struct DriveParams {
    float track_width;
    float wheel_radius;

    float mass;             // literally doesn't matter right now
    float inertia;          // literally doesn't matter right now
    float max_velocity;     // linear velocity of the vehicle, meters per second
    float max_omega;        // angular velocity of the wheels, radians per second

    DriveParams()
        :track_width(0.5f),
        wheel_radius(0.5f),
        mass(1.0),
        inertia(1.0),
        max_velocity(1.0),
        max_omega(1.0f) {}
};

class VehicleDynamics{
    private:
        float vL, vR;
    public:
        VehicleDynamics(const DriveParams& params);
        ~VehicleDynamics();

        VehicleState integrate(VehicleState current_state, Vector2d control, float dt);
        Vector3d predictPose(Vector3d& current_pose, float v, float omega, float dt);
        Vector3d updateFOV(Vector3d& current_pose);

        private:
            DriveParams params_;
            Vector3d integratePose(const Vector3d& pose, float v, float omega, float dt) const;

};

class OpenCVVisualizer {
    private:
        int width;
        int height;
        float scale; // pixels per meter
        cv::Mat canvas;
    public:
        OpenCVVisualizer(int width, int height, float scale);
        ~OpenCVVisualizer();

        void clear();
        void drawTrail(const std::vector<Vector2d>& trail);
        void drawRobot(float x, float y, float theta);
        void drawFOV(float x, float y, float theta_left, float theta_right, float depth);
        void render();
};



} // namespace vehicle_dynamics