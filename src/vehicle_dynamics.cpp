#include "vehicle_dynamics.hpp"

namespace vehicle_dynamics {

// Contructor and destructor
    VehicleDynamics::VehicleDynamics(const DriveParams& params) : params_(params) {};

    VehicleDynamics::~VehicleDynamics() {}

// Rest of the functions
    VehicleState VehicleDynamics::integrate(VehicleState current_state, Vector2d control, float dt)
        {
        // Tank Drive model for the robot
        vL = control(0) * params_.max_omega * params_.wheel_radius; // convert from control input to linear velocity
        vR = control(1) * params_.max_omega * params_.wheel_radius; // convert from control input to linear velocity
        float v = (vL + vR) / 2.0f;
        float omega = (vR - vL) / params_.track_width;

        VehicleState new_state;
        new_state.pose = predictPose(current_state.pose, v, omega, dt);
        new_state.fov = updateFOV(new_state.pose);
        new_state.vel = Vector2d(v * std::cos(new_state.pose(2)), v * std::sin(new_state.pose(2))); // velocity in world frame
        new_state.omega = omega;
        // these accelerations might not be useful at all
        // might be worth it later to estimate bias or drift or any sort of alpha-beta filter implementation
        new_state.a = (v - current_state.vel.norm()) / dt; // simple finite difference for acceleration
        new_state.alpha = (omega - current_state.alpha) / dt;

        return new_state;
    };

    Vector3d VehicleDynamics::predictPose(Vector3d& pose, float v, float omega, float dt)
    {
        // simple integration of the pose tank drive
        // coordinates for pose is given in the world frame
        Vector3d prediction = pose;

        prediction(0) += v * std::cos(pose(2)) * dt;
        prediction(1) += v * std::sin(pose(2)) * dt;
        prediction(2) += omega * dt; // units will be in radians.

        return prediction;
    };

    Vector3d VehicleDynamics::updateFOV(Vector3d& pose)
    {
        // Will need to update with real camera,
        Vector3d fov; // structure of [Left, Right, Depth]
        fov(0) = pose[2] + 1;
        fov(1) = pose[2] - 1;
        fov(2) = 30;
        return fov;
    };

// Visualizer OpenCV

    OpenCVVisualizer::OpenCVVisualizer(int width, int height, float scale) {
        // Initialize the OpenCV visualizer
        // width and height are in pixels, scale is how many pixels per meter
        this->width = width;
        this->height = height;
        this->scale = scale;
        canvas = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    OpenCVVisualizer::~OpenCVVisualizer() {}

    void OpenCVVisualizer::clear() {
        canvas = cv::Scalar(0, 0, 0);
    }

    void OpenCVVisualizer::drawTrail(const std::vector<Vector2d>& trail) {
        for (size_t i = 1; i < trail.size(); i++) {
            cv::line(canvas,
                     cv::Point(trail[i-1](0) * scale + width / 2, height / 2 - trail[i-1](1) * scale),
                     cv::Point(trail[i](0) * scale + width / 2, height / 2 - trail[i](1) * scale),
                     cv::Scalar(255, 255, 255), 2);
        }
    }

    void OpenCVVisualizer::drawRobot(float x, float y, float theta) {
        // Draw a simple triangle to represent the robot
        std::vector<cv::Point> points;
        points.push_back(cv::Point(x * scale + width / 2 + 10 * std::cos(theta), height / 2 - y * scale - 10 * std::sin(theta)));
        points.push_back(cv::Point(x * scale + width / 2 + 10 * std::cos(theta + M_PI / 2), height / 2 - y * scale - 10 * std::sin(theta + M_PI / 2)));
        points.push_back(cv::Point(x * scale + width / 2 + 10 * std::cos(theta - M_PI / 2), height / 2 - y * scale - 10 * std::sin(theta - M_PI / 2)));

        cv::fillConvexPoly(canvas, points.data(), points.size(), cv::Scalar(0, 255, 0));
    }

    void OpenCVVisualizer::drawFOV(float x, float y, float theta_left, float theta_right, float depth) {
        // Convert robot position to screen coordinates
        int cx = x * scale + width / 2;
        int cy = height / 2 - y * scale;
        
        // Calculate FOV boundary endpoints
        int left_x = cx + depth * scale * std::cos(theta_left);
        int left_y = cy - depth * scale * std::sin(theta_left);
        
        int right_x = cx + depth * scale * std::cos(theta_right);
        int right_y = cy - depth * scale * std::sin(theta_right);
        
        // Draw left boundary line
        cv::line(canvas, cv::Point(cx, cy), cv::Point(left_x, left_y), 
                cv::Scalar(255, 255, 255), 1);
        
        // Draw right boundary line
        cv::line(canvas, cv::Point(cx, cy), cv::Point(right_x, right_y), 
                cv::Scalar(255, 255, 255), 1);
        
        // Optional: Draw arc at the end to show FOV range
        cv::ellipse(canvas, cv::Point(cx, cy), 
                    cv::Size(depth * scale, depth * scale),
                    0, 
                    -theta_right * 180 / M_PI,  // Convert to degrees
                    -theta_left * 180 / M_PI, 
                    cv::Scalar(255, 255, 255, 50), 1);
    }

    void OpenCVVisualizer::render() {
        cv::imshow("Vehicle Simulation", canvas);
        cv::waitKey(1);
    }

} // namespace vehicle_dynamics
