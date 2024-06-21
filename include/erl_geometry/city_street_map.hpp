#pragma once

#include "erl_common/opencv.hpp"

namespace erl::geometry {

    /**
     * Load map data from 2D pathfinding benchmark files.
     * @ref https://www.movingai.com/benchmarks/street/index.html
     */
    struct CityStreetMap {

        static constexpr int kFree = 0;
        static constexpr int kObstacle = 255;

        static constexpr char kPassableDot = '.';
        static constexpr char kPassableG = 'G';
        static constexpr char kOutOfBoundAt = '@';
        static constexpr char kOutOfBoundO = 'O';
        static constexpr char kTree = 'T';
        static constexpr char kSwamp = 'S';
        static constexpr char kWater = 'W';

        struct Scene {
            int bucket;
            std::string map;
            int map_width;
            int map_height;
            int start_x;
            int start_y;
            int goal_x;
            int goal_y;
            double optimal_length;
        };

        static cv::Mat
        LoadMap(const std::string &filename);

        static std::vector<Scene>
        LoadScenes(const std::string &filename);
    };

}  // namespace erl::geometry
