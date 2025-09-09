#include "erl_common/test_helper.hpp"
#include "erl_geometry/libmorton_torch.hpp"

TEST(libmorton, Torch) {

    using namespace erl::geometry;
    torch::Tensor codes = torch::randint(0, 1 << 16, {100000}, torch::kUInt32);

    torch::Tensor coords_2d_cpu;
    MortonDecodeTorch(codes, 2, coords_2d_cpu);
    torch::Tensor coords_2d_cuda;
    MortonDecodeTorch(codes.cuda(), 2, coords_2d_cuda);
    ASSERT_TRUE(coords_2d_cpu.equal(coords_2d_cuda.cpu()));

    torch::Tensor coords_3d_cpu;
    MortonDecodeTorch(codes, 3, coords_3d_cpu);
    torch::Tensor coords_3d_cuda;
    MortonDecodeTorch(codes.cuda(), 3, coords_3d_cuda);
    ASSERT_TRUE(coords_3d_cpu.equal(coords_3d_cuda.cpu()));

    torch::Tensor codes_cpu;
    MortonEncodeTorch(coords_2d_cpu, codes_cpu);
    torch::Tensor codes_cuda;
    MortonEncodeTorch(coords_2d_cuda, codes_cuda);
    ASSERT_TRUE(codes_cpu.equal(codes_cuda.cpu()));

    MortonEncodeTorch(coords_3d_cpu, codes_cpu);
    MortonEncodeTorch(coords_3d_cuda, codes_cuda);
    ASSERT_TRUE(codes_cpu.equal(codes_cuda.cpu()));
}
