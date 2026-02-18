#ifdef ERL_USE_LIBTORCH

    #include "erl_common/block_timer.hpp"
    #include "erl_common/test_helper.hpp"
    #include "erl_geometry/libmorton_torch.hpp"

    #include <cuda_runtime_api.h>

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

    // measure performance
    constexpr long n_repeats = 10;
    double dt_cpu_encode_2d = 0;
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton encode (2D, CPU)", dt_cpu_encode_2d);
        for (long i = 0; i < n_repeats; ++i) { MortonEncodeTorch(coords_2d_cpu, codes_cpu); }
    }

    double dt_cuda_encode_2d = 0;
    cudaDeviceSynchronize();
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton encode (2D, CUDA)", dt_cuda_encode_2d);
        for (long i = 0; i < n_repeats; ++i) {
            MortonEncodeTorch(coords_2d_cuda, codes_cuda);
            cudaDeviceSynchronize();
        }
    }

    double dt_cpu_decode_2d = 0;
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton decode (2D, CPU)", dt_cpu_decode_2d);
        for (long i = 0; i < n_repeats; ++i) { MortonDecodeTorch(codes_cpu, 2, coords_2d_cpu); }
    }

    double dt_cuda_decode_2d = 0;
    cudaDeviceSynchronize();
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton decode (2D, CUDA)", dt_cuda_decode_2d);
        for (long i = 0; i < n_repeats; ++i) {
            MortonDecodeTorch(codes_cuda, 2, coords_2d_cuda);
            cudaDeviceSynchronize();
        }
    }

    double dt_cpu_encode_3d = 0;
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton encode (3D, CPU)", dt_cpu_encode_3d);
        for (long i = 0; i < n_repeats; ++i) { MortonEncodeTorch(coords_3d_cpu, codes_cpu); }
    }

    double dt_cuda_encode_3d = 0;
    cudaDeviceSynchronize();
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton encode (3D, CUDA)", dt_cuda_encode_3d);
        for (long i = 0; i < n_repeats; ++i) {
            MortonEncodeTorch(coords_3d_cuda, codes_cuda);
            cudaDeviceSynchronize();
        }
    }

    double dt_cpu_decode_3d = 0;
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton decode (3D, CPU)", dt_cpu_decode_3d);
        for (long i = 0; i < n_repeats; ++i) { MortonDecodeTorch(codes_cpu, 3, coords_3d_cpu); }
    }

    double dt_cuda_decode_3d = 0;
    cudaDeviceSynchronize();
    {
        const ERL_BLOCK_TIMER_MICRO_MSG_TIME("morton decode (3D, CUDA)", dt_cuda_decode_3d);
        for (long i = 0; i < n_repeats; ++i) {
            MortonDecodeTorch(codes_cuda, 3, coords_3d_cuda);
            cudaDeviceSynchronize();
        }
    }

    dt_cpu_encode_2d /= static_cast<double>(n_repeats);
    dt_cuda_encode_2d /= static_cast<double>(n_repeats);
    dt_cpu_decode_2d /= static_cast<double>(n_repeats);
    dt_cuda_decode_2d /= static_cast<double>(n_repeats);
    dt_cpu_encode_3d /= static_cast<double>(n_repeats);
    dt_cuda_encode_3d /= static_cast<double>(n_repeats);
    dt_cpu_decode_3d /= static_cast<double>(n_repeats);
    dt_cuda_decode_3d /= static_cast<double>(n_repeats);

    ERL_INFO("Performance Summary for {} coords:", codes.size(0));
    ERL_INFO(
        "Morton Encode (2D): CPU: {} us, CUDA: {} us, ratio: {}",
        dt_cpu_encode_2d,
        dt_cuda_encode_2d,
        dt_cpu_encode_2d / dt_cuda_encode_2d);
    ERL_INFO(
        "Morton Decode (2D): CPU: {} us, CUDA: {} us, ratio: {}",
        dt_cpu_decode_2d,
        dt_cuda_decode_2d,
        dt_cpu_decode_2d / dt_cuda_decode_2d);
    ERL_INFO(
        "Morton Encode (3D): CPU: {} us, CUDA: {} us, ratio: {}",
        dt_cpu_encode_3d,
        dt_cuda_encode_3d,
        dt_cpu_encode_3d / dt_cuda_encode_3d);
    ERL_INFO(
        "Morton Decode (3D): CPU: {} us, CUDA: {} us, ratio: {}",
        dt_cpu_decode_3d,
        dt_cuda_decode_3d,
        dt_cpu_decode_3d / dt_cuda_decode_3d);
}

#endif
