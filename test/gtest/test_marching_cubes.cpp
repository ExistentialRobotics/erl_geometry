#include "erl_common/test_helper.hpp"
#include "erl_geometry/marching_cubes.hpp"

#include <map>

TEST(MarchingCubes, Basic) {
    using namespace erl::geometry;
    std::map<long, std::size_t> unique_indices;
    constexpr int n = 14;
    for (long i = 0; i < 256; ++i) {
        unique_indices.clear();
        for (long j = 0; j < 16; ++j) {
            if (MarchingCubes::kTriangleEdgeIndexTable[i][j] == -1) {
                std::cout << j / 3 << ", ";
                break;
            }
            unique_indices.try_emplace(
                MarchingCubes::kTriangleEdgeIndexTable[i][j],
                unique_indices.size());
        }
        // std::cout << fmt::format(
        //                  "i: {:02d}, {} unique indices: {}",
        //                  i,
        //                  unique_indices.size(),
        //                  unique_indices)
        //           << std::endl;

        // std::cout << '{';
        // for (auto [j, k]: unique_indices) {
        //     if (j == -1) { continue; }
        //     std::cout << j << ", ";
        // }
        // for (long j = 0; j < n - unique_indices.size(); ++j) { std::cout << -1 << ", "; }
        // std::cout << "-1}," << std::endl;

        // std::cout << '{';
        // long m = 0;
        // for (long j = 0; j < 16; ++j) {
        //     long k = MarchingCubes::kTriangleEdgeIndexTable[i][j];
        //     auto it = unique_indices.find(k);
        //     if (it == unique_indices.end()) {
        //         std::cout << -1;
        //         break;
        //     }
        //     std::cout << it->second << ", ";
        //     ++m;
        // }
        // for (long j = 1; j < 16 - m; ++j) { std::cout << ", -1"; }
        // std::cout << "}," << std::endl;
    }
}
