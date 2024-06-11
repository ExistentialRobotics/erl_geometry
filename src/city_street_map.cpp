#include "erl_geometry/city_street_map.hpp"

#include <fstream>

namespace erl::geometry {

    cv::Mat
    CityStreetMap::Load(const std::string &filename) {
        std::ifstream file(filename);
        ERL_ASSERTM(file.is_open(), "Failed to open file: {}", filename);

        std::string token;
        file >> token;
        ERL_ASSERTM(token == "type", "Invalid file: {}", filename);
        file >> token;
        ERL_ASSERTM(token == "octile", "Invalid file: {}", filename);

        file >> token;
        ERL_ASSERTM(token == "height", "Invalid file: {}", filename);
        int height;
        file >> height;

        file >> token;
        ERL_ASSERTM(token == "width", "Invalid file: {}", filename);
        int width;
        file >> width;

        file >> token;
        ERL_ASSERTM(token == "map", "Invalid file: {}", filename);

        std::string line;
        file >> line;
        cv::Mat map(height, width, CV_8UC1);
        int rows = 0;
        while (!line.empty()) {
            ERL_ASSERTM(
                line.size() == static_cast<std::size_t>(width),
                "Line {} of {} is incomplete: line.size() is {} != {}",
                rows,
                filename,
                line.size(),
                width);
            for (int cols = 0; cols < width; ++cols) {
                switch (line[cols]) {
                    case kPassableDot:
                    case kPassableG:
                        map.at<uchar>(rows, cols) = kFree;
                        break;
                    case kOutOfBoundAt:
                    case kOutOfBoundO:
                    case kTree:
                    case kSwamp:
                    case kWater:
                        map.at<uchar>(rows, cols) = kObstacle;
                        break;
                    default:
                        ERL_ASSERTM(false, "Invalid character {} at ({}, {}) in {}", line[cols], rows, cols, filename);
                }
            }
            ++rows;
            line.clear();
            file >> line;
        }
        ERL_ASSERTM(rows == height, "Height mismatch: {} != {}", rows, height);
        return map;
    }

}  // namespace erl::geometry
