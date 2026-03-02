#pragma once

#include <cmath>
#include <vector>
#include <utility>
#include "vehicle_dynamics.hpp"
#include "Utilities.hpp"

static constexpr float LEARNING_RATE = 0.5f;

using namespace utilities;

struct OccupancyCell {
    float prob;
    OccupancyCell() : prob(0.5f) {};  // default everything is set to 0.5 (unknown)

    void updateCell(float p) {prob += LEARNING_RATE * (p - prob);}
    float getCell() {return prob;}
    bool isOccupied() {return prob > 0.7f;}
    bool isFree() {return prob < 0.3f;}
    bool isUnknown() {return prob >= 0.3f && prob <= 0.7f;}
};

class OccupancyGrid {
    /*
    A simple grid implementation, each cell uses Occupancy Cell implementation, probability of occupancy 0-1
    Think the best thing to do is to have a local grid that is centered around the robot and transform the local grid
    to the global using pose.
    Keeps memory usage to a minimum and keeps resizing a more refined map outside of this local grid.
    This is more simple used for local obstacle avoidance and planning, global map used for global planning and loop closure.
    separating the two should make this easier to maintain and change. 
    ie I like modularity.... 
    */
    public:

        OccupancyGrid(int w, int h, float res, float origin_x = 0.0f, float origin_y = 0.0f);
        ~OccupancyGrid() = default;

        void updateFromPoint(Vector3d robot_pose, Vector2d point);
        void updateFromScan(Vector3d robot_pose, std::vector<Vector2d> scanpoints);

        void setCellProb(int x, int y, float prob);


        void clear();
        void clearRegion(int x_min, int x_max, int y_min, int y_max); // should be a square? circle? idc ab polygons jsut yet

        // Queries
        OccupancyCell& getCell(int x, int y) const;
        int getOccupiedCount() const;
        int getFreeCount() const;
        int getUnknownCount() const;

        bool isCellOccupied(int x, int y) const;

        int getWidthCells() const { return width; }
        int getHeightCells() const { return height; }
        float getResolution() const { return resolution; }
        float getOriginX() const { return origin_x; }
        float getOriginY() const { return origin_y; }

        // cool visualizations
        void savetoFile();
    private:

        static int width;           // in meters
        static int height;          // in meters
        float resolution;           // cells size in meters
        int origin_x;               // in cells
        int origin_y;               // in cells
        std::vector<std::vector<OccupancyCell>> data_;

        // this might be up for discussion, but claude suggested instead of having a 2d vector
        // of cells, make it 1d and access from it using this index function
        int index(int grid_x, int grid_y) const {
            return grid_y * width + grid_x;
        }

};