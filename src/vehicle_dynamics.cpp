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

// Visualizer

    ASCIIVisualizer::ASCIIVisualizer(int width = 80, int height = 40, float scale = 5.0f)
        : width_(width), height_(height), scale_(scale) {
        buffer_.resize(width * height, ' ');
    }
    
    void ASCIIVisualizer::clear() {
        std::fill(buffer_.begin(), buffer_.end(), ' ');
    }
    
    void ASCIIVisualizer::drawPoint(float x, float y, char symbol = '*') {
        // Convert world coordinates to screen coordinates
        int sx = static_cast<int>(width_ / 2 + x * scale_);
        int sy = static_cast<int>(height_ / 2 - y * scale_);  // flip y
        
        if (sx >= 0 && sx < width_ && sy >= 0 && sy < height_) {
            buffer_[sy * width_ + sx] = symbol;
        }
    }
    
    void ASCIIVisualizer::drawRobot(float x, float y, float theta) {
        drawPoint(x, y, 'O');
        
        // Draw orientation arrow
        float arrow_length = 0.3f;
        float tip_x = x + arrow_length * std::cos(theta);
        float tip_y = y + arrow_length * std::sin(theta);
        drawPoint(tip_x, tip_y, '>');
    }
    
    void ASCIIVisualizer::drawTrail(const std::vector<Vector2d>& trail) {
        for (const auto& point : trail) {
            drawPoint(point(0), point(1), '.');
        }
    }
    
    void ASCIIVisualizer::render() {
        // Clear screen (ANSI escape code)
        std::cout << "\033[2J\033[H";
        
        // Draw top border
        std::cout << "+";
        for (int i = 0; i < width_; ++i) std::cout << "-";
        std::cout << "+\n";
        
        // Draw buffer
        for (int y = 0; y < height_; ++y) {
            std::cout << "|";
            for (int x = 0; x < width_; ++x) {
                std::cout << buffer_[y * width_ + x];
            }
            std::cout << "|\n";
        }
        
        // Draw bottom border
        std::cout << "+";
        for (int i = 0; i < width_; ++i) std::cout << "-";
        std::cout << "+\n";
    }
};
