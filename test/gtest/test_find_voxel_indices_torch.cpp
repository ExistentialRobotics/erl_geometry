#ifdef ERL_USE_LIBTORCH

    #include "erl_common/block_timer.hpp"
    #include "erl_common/test_helper.hpp"
    #include "erl_geometry/find_voxel_indices_torch.hpp"
    #include "erl_geometry/libmorton_torch.hpp"

    #include <cuda_runtime_api.h>

template<int Dim>
void
TestFindVoxelIndicesTorch() {
    using namespace erl::common;
    const std::filesystem::path data_dir = ERL_GEOMETRY_ROOT_DIR "/data";

    std::filesystem::path points_fp =
        data_dir / (Dim == 3 ? "semi_sparse_octree_points.bin" : "semi_sparse_quadtree_points.bin");
    std::vector<double> points = LoadBinaryFile<double>(points_fp);
    const auto n_points = points.size() / Dim;

    std::filesystem::path children_fp = data_dir / (Dim == 3 ? "semi_sparse_octree_children.bin"
                                                             : "semi_sparse_quadtree_children.bin");
    std::vector<long> children = LoadBinaryFile<long>(children_fp);
    const auto n_nodes = children.size() / (1 << Dim);

    std::filesystem::path node_indices_fp =
        data_dir / (Dim == 3 ? "semi_sparse_octree_node_indices.bin"
                             : "semi_sparse_quadtree_node_indices.bin");
    std::vector<long> node_indices = LoadBinaryFile<long>(node_indices_fp);
    ASSERT_EQ(n_points, node_indices.size());
    torch::Tensor node_indices_gt =
        torch::from_blob(node_indices.data(), {static_cast<long>(n_points)}, torch::kInt64);

    using namespace erl::geometry;
    constexpr double resolution = 0.05;
    constexpr int level = 7;
    constexpr uint32_t coord_offset = 1 << level;
    torch::Tensor points_tensor =
        torch::from_blob(points.data(), {static_cast<long>(n_points), Dim}, torch::kFloat64)
            .clone();
    torch::Tensor coords = (points_tensor / resolution).floor().to(torch::kInt32) + coord_offset;
    coords = coords.to(torch::kUInt32);
    torch::Tensor codes;
    MortonEncodeTorch(coords, codes);
    torch::Tensor children_tensor =
        torch::from_blob(children.data(), {static_cast<long>(n_nodes), 1 << Dim}, torch::kInt64)
            .clone();

    // test CPU version
    torch::Tensor voxel_indices;
    double dt_cpu;
    {
        ERL_BLOCK_TIMER_MSG_TIME("find voxel indices (CPU)", dt_cpu);
        FindVoxelIndicesTorch(codes, Dim, level, children_tensor, true, voxel_indices);
    }
    dt_cpu /= static_cast<double>(n_points);
    std::cout << dt_cpu * 1e3 << " us per query (CPU)." << std::endl;
    ASSERT_TRUE(voxel_indices.equal(node_indices_gt));

    // test CUDA version
    double dt_cuda;
    codes = codes.cuda();
    children_tensor = children_tensor.cuda();
    cudaDeviceSynchronize();
    {
        ERL_BLOCK_TIMER_MSG_TIME("find voxel indices (CUDA)", dt_cuda);
        FindVoxelIndicesTorch(codes, Dim, level, children_tensor, true, voxel_indices);
        cudaDeviceSynchronize();
    }
    dt_cuda /= static_cast<double>(n_points);
    std::cout << dt_cuda * 1e3 << " us per query (CUDA)." << std::endl;
    ASSERT_TRUE(voxel_indices.cpu().equal(node_indices_gt));
}

TEST(FindVoxelIndices, Torch) {
    TestFindVoxelIndicesTorch<2>();
    TestFindVoxelIndicesTorch<3>();
}

#endif
