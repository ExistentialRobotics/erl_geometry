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

    constexpr long n_repeats = 10;

    // test CPU version
    torch::Tensor voxel_indices;
    double dt_cpu = 0.0;
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("find voxel indices (CPU)", dt_cpu);
        for (long i = 0; i < n_repeats; ++i) {
            FindVoxelIndicesTorch(codes, Dim, level, children_tensor, true, voxel_indices);
        }
    }
    dt_cpu /= static_cast<double>(n_repeats);
    std::cout << dt_cpu << " us (CPU)." << std::endl;
    ASSERT_TRUE(voxel_indices.equal(node_indices_gt));

    // test CUDA version
    double dt_cuda = 0.0;
    codes = codes.cuda();
    children_tensor = children_tensor.cuda();
    cudaDeviceSynchronize();
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("find voxel indices (CUDA)", dt_cuda);
        for (long i = 0; i < n_repeats; ++i) {
            FindVoxelIndicesTorch(codes, Dim, level, children_tensor, true, voxel_indices);
            cudaDeviceSynchronize();
        }
    }
    dt_cuda /= static_cast<double>(n_repeats);
    std::cout << dt_cuda << " us (CUDA)." << std::endl;
    ASSERT_TRUE(voxel_indices.cpu().equal(node_indices_gt));

    const double ratio = dt_cpu / dt_cuda;
    std::cout << "Speedup (CPU / CUDA): " << ratio << "x." << std::endl;
}

TEST(FindVoxelIndices, Torch) {
    ERL_INFO("GPU Warmup...");
    TestFindVoxelIndicesTorch<2>();
    ERL_INFO("Ignore the above timing for GPU warmup.");

    TestFindVoxelIndicesTorch<2>();
    TestFindVoxelIndicesTorch<3>();
}

#endif
