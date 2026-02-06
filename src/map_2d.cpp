/*
Implementation of the 2d map

Date: Feb 6, 2026
*/

#include "map_2d.hpp"

OccupancyGrid::OccupancyGrid(int w, int h, float res, float origin_x = 0.0f, float origin_y = 0.0f)
    : resolution(res),
    origin_x(origin_x),
    origin_y(origin_y)
{
    width = static_cast<int>(w / res);
    height = static_cast<int>(h / res);
    int total_cells = width * height;
    data_.resize(total_cells);
}

OccupancyGrid::~OccupancyGrid()
{
    clear();
}

OccupancyCell& OccupancyGrid::getCell(int x, int y) {
    return data_[index(x, y)];
}

void OccupancyGrid::clear() {
    for (auto& cell : data_) {
        cell.prob = 0.5f;  // Reset to unknown
    }
}

void OccupancyGrid::clearRegion(int x_min, int x_max, int y_min, int y_max) {
    // Clamp to grid bounds
    x_min = std::max(0, x_min);
    x_max = std::min(width - 1, x_max);
    y_min = std::max(0, y_min);
    y_max = std::min(height - 1, y_max);

    for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
            getCell(x, y).prob = 0.5f;
        }
    }
}

int OccupancyGrid::getOccupiedCount() const {
    int count = 0;
    for (const auto& cell : data_) {
        if (cell.isOccupied()) {
            ++count;
        }
    }
    return count;
}

int OccupancyGrid::getFreeCount() const {
    int count = 0;
    for (const auto& cell : data_) {
        if (cell.isFree()) {
            ++count;
        }
    }
    return count;
}

int OccupancyGrid::getUnknownCount() const {
    int count = 0;
    for (const auto& cell : data_) {
        if (cell.isUnknown()) {
            ++count;
        }
    }
    return count;
}

bool OccupancyGrid::isCellOccupied(int x, int y) const {
    if (!isInBounds(x, y)) {
        return false;  // Out of bounds treated as free
    }
    return getCell(x, y).isOccupied();
}

// updates from camera sensor scan:
void OccupancyGrid::updateFromPoint(Pose2D robot_pose, Point2D point) {
    int sensor_x = static_cast<int>(robot_pose.x);
    int sensor_y = static_cast<int>(robot_pose.y);
    int obs_x = static_cast<int>(point.x);
    int obs_y = static_cast<int>(point.y);


    // Need to implement ray tracing
    auto ray_cells = rayTrace(sensor_x, sensor_y, obs_x, obs_y);

    // Update all cells along ray (except last) as FREE
    for (size_t i = 0; i < ray_cells.size() - 1; ++i) {
        int x = ray_cells[i].first;
        int y = ray_cells[i].second;
    }
    
    // Update endpoint as OCCUPIED
    getCell(obs_x, obs_y).updateCell(0.9f);  // High prob = occupied
}

void OccupancyGrid::updateFromScan(Pose2D robot_pose, std::vector<Point2D> scanpoints) {
    for (const auto& point : scanpoints) {
        updateFromPoint(robot_pose, point);
    }
}


// NGL this was full gpt
bool OccupancyGrid::saveToFile(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }
    
    // Write metadata
    file.write(reinterpret_cast<const char*>(&width), sizeof(width));
    file.write(reinterpret_cast<const char*>(&height), sizeof(height));
    file.write(reinterpret_cast<const char*>(&resolution), sizeof(resolution));
    file.write(reinterpret_cast<const char*>(&origin_x), sizeof(origin_x));
    file.write(reinterpret_cast<const char*>(&origin_y), sizeof(origin_y));
    
    // Write cell data
    for (const auto& cell : data_) {
        file.write(reinterpret_cast<const char*>(&cell.prob), sizeof(cell.prob));
    }
    
    file.close();
    return true;
}