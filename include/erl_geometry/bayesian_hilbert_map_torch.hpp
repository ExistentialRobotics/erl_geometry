#pragma once

#ifdef ERL_USE_LIBTORCH

    #include "bayesian_hilbert_map.hpp"

    #include <torch/torch.h>

    #include <memory>
    #include <vector>

namespace erl::geometry {

    /**
     * Update a set of Bayesian Hilbert maps using libtorch. This class is expected to be used with
     * BayesianHilbertMap where weights and covariance are stored as Eigen matrices or vectors.
     * Data are loaded from set of BayesianHilbertMap instances to this class, and then the
     * computations are performed on the GPU when the device is set to CUDA. For compatibility on
     * devices where NVIDIA GPU is unavailable, this class also works on CPU if you set the device
     * to torch::kCPU. This class is designed to speed up the update. For prediction, the
     * implementation is also provided. However, it is not batched because the number of points for
     * each map can be very different. This means that the prediction code may be slower than the
     * BayesianHilbertMap implementation, which is also per map prediction but can be parallelized.
     */
    template<typename Dtype, int Dim>
    class BayesianHilbertMapTorch {
    public:
        using MatrixDX = Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>;
        using MatrixX = Eigen::MatrixX<Dtype>;
        using VectorD = Eigen::Vector<Dtype, Dim>;
        using VectorX = Eigen::VectorX<Dtype>;

    private:
        std::shared_ptr<geometry::BayesianHilbertMapSetting> m_setting_ = nullptr;
        // (1, M, Dim) matrix of hinged points
        torch::Tensor m_hinged_points_;
        // (B, M + 1, M + 1) matrix of the sigma inverse, or (B, M) if diagonal_sigma is true
        torch::Tensor m_sigma_inv_;
        // (B, M + 1, M + 1) matrix of sigma
        torch::Tensor m_sigma_;
        // (B, M + 1, 1) matrix, each row is sigma_inv @ mu
        torch::Tensor m_alpha_;
        // (B, M + 1, 1) matrix of weights
        torch::Tensor m_mu_;
        // (B, 1, Dim) matrix of map centers, each row is the center of the map
        torch::Tensor m_map_centers_;
        // (B, 1, Dim) matrix of map centers, but on CPU, used for loading the dataset
        torch::Tensor m_map_centers_cpu_;
        // (B, N, Dim) matrix of the points in the dataset, each row is a point in the dataset
        torch::Tensor m_points_;
        // (B, N, Dim) matrix of the points in the dataset, but on CPU, used for loading the dataset
        torch::Tensor m_points_cpu_;
        // (B, N, 1) matrix of labels, each row is the input label of the dataset for each map
        torch::Tensor m_labels_;
        // (B, N, 1) matrix of labels, but on CPU, used for loading the dataset
        torch::Tensor m_labels_cpu_;
        // (B, M + 1, N) matrix of the feature matrix, each row is the feature matrix for each map.
        // If `use_sparse` is true, this is a sparse matrix of shape (B * (M + 1), B * N).
        torch::Tensor m_phi_;
        // (B, M + 1, N) matrix of the squared feature matrix.
        // If `use_sparse` is true, this is a sparse matrix of shape (B * (M + 1), B * N).
        torch::Tensor m_phi_sq_;
        // (B, N, 1) matrix of the lower bound parameters xi, each row is the xi vector for each map
        torch::Tensor m_xi_;
        // (B, N, 1) matrix of the lambda vectors, each row is the lambda vector for each map
        torch::Tensor m_lambda_;
        // B: number of maps
        long m_num_maps_ = 0;
        // N: number of points in the dataset of each map
        long m_num_points_ = 0;
        // number of dataset points for each map
        std::vector<long> m_num_loaded_points_;
        // flags to indicate if the dataset is loaded for each map
        std::vector<char> m_weights_loaded_;
        // flag to indicate if the weights have changed
        bool m_weights_changed_ = false;
        // device to run the computations on, default is CUDA
        torch::Device m_device_ = torch::kCUDA;
        // kernel scale parameter
        Dtype m_kernel_scale_ = 1.0;

    public:
        BayesianHilbertMapTorch() = delete;

        explicit BayesianHilbertMapTorch(
            std::shared_ptr<geometry::BayesianHilbertMapSetting> setting);

        void
        SetDevice(const torch::Device &device) {
            m_device_ = device;
        }

        void
        SetKernelScale(Dtype scale) {
            m_kernel_scale_ = scale;
        }

        /**
         *
         * @param hinged_points Eigen matrix of hinged points, shape (Dim, M).
         */
        void
        LoadHingedPoints(const MatrixDX &hinged_points);

        /**
         *
         * @param num_maps Number of Bayesian Hilbert Maps to create.
         * @param num_points Number of points in the dataset of each map.
         */
        void
        PrepareMemory(long num_maps, long num_points);

        void
        LoadWeightsAndCovariance(long map_idx, const VectorX &weights, const MatrixX &cov);

        void
        GetWeightsAndCovariance(long map_idx, VectorX &weights, MatrixX &cov);

        void
        LoadDataset(
            long map_idx,
            const VectorD &map_center,
            const MatrixDX &points,
            const VectorX &labels,
            long num_points);

        void
        RunExpectationMaximization();

        void
        PrepareExpectationMaximization();

        void
        RunExpectationMaximizationIteration();

        void
        RunExpectationMaximizationIterationSparse();

        void
        FinishExpectationMaximization();

        void
        ToCpu();

        void
        Reset();

        void
        Predict(
            long map_idx,
            const Eigen::Ref<const MatrixDX> &points,
            bool logodd,
            bool faster,
            bool compute_gradient,
            bool gradient_with_sigmoid,
            VectorX &prob_occupied,
            MatrixDX &gradient) const;

    private:
        // (B, M, N) to (B * M, B * N)
        [[nodiscard]] torch::Tensor
        ToBlockDiagonal(torch::Tensor indices, const torch::Tensor &values, long b, long m, long n)
            const;

        [[nodiscard]] torch::Tensor
        ToBatched(torch::Tensor indices, const torch::Tensor &values, long b, long m, long n) const;

        [[nodiscard]] torch::Tensor
        CollapseDim(
            torch::Tensor indices,
            const torch::Tensor &values,
            long b,
            long m,
            long n,
            long dim) const;
    };

    extern template class BayesianHilbertMapTorch<float, 2>;
    extern template class BayesianHilbertMapTorch<float, 3>;
    extern template class BayesianHilbertMapTorch<double, 2>;
    extern template class BayesianHilbertMapTorch<double, 3>;

}  // namespace erl::geometry
#endif
