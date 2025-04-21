#pragma once

#include "aabb.hpp"

#include "erl_common/yaml.hpp"
#include "erl_covariance/covariance.hpp"

#include <Eigen/Sparse>

namespace erl::geometry {

    struct BayesianHilbertMapSetting : common::Yamlable<BayesianHilbertMapSetting> {
        bool diagonal_sigma = false;          // if true, the covariance matrix will be assumed to be diagonal to speed up the computation.
        float max_distance = 30.0f;           // maximum distance from the sensor to consider a point as occupied.
        float free_points_per_meter = 2.0f;   // number of free points to sample per meter from the sensor.
        float free_sampling_margin = 0.05f;   // percentage margin to use when sampling free points to avoid sampling too close to the surface or the sensor.
        float init_sigma = 1.0e4f;            // initial value for initializing the covariance matrix.
        int num_em_iterations = 3;            // number of iterations for the Expectation-Maximization (EM) algorithm to optimize the mean and covariance.
        float sparse_zero_threshold = 1e-6f;  // threshold for sparse matrix zeroing.
        bool use_sparse = false;              // if true, use sparse matrix for the feature matrix.
    };

    template<typename Dtype, int Dim>
    class BayesianHilbertMap {
    public:
        using Covariance = covariance::Covariance<Dtype>;
        using MatrixDX = Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>;
        using MatrixX = Eigen::MatrixX<Dtype>;
        using VectorD = Eigen::Vector<Dtype, Dim>;
        using VectorX = Eigen::VectorX<Dtype>;
        using SparseMatrix = Eigen::SparseMatrix<Dtype>;

    private:
        std::shared_ptr<BayesianHilbertMapSetting> m_setting_ = nullptr;  // settings for the Bayesian Hilbert Map
        std::shared_ptr<Covariance> m_kernel_ = nullptr;
        MatrixDX m_hinged_points_{};  // [Dim, N] hinged points for computing the hilbert space features
        Aabb<Dtype, Dim> m_map_boundary_{};
        std::mt19937_64 m_generator_;
        uint64_t m_iteration_cnt_ = 0;              // iteration count for the EM algorithm (just for statistics)
        MatrixX m_sigma_inv_{};                     // inverse of the posterior weight covariance
        MatrixX m_sigma_{};                         // posterior covariance of the weights
        MatrixX m_sigma_inv_mat_l_{};               // matrix L for Cholesky decomposition of m_sigma_inv_, used when diagonal_sigma is false
        VectorX m_alpha_{};                         // alpha = sigma_inv * mu
        VectorX m_labels_{};                        // label = input_label - 0.5
        VectorX m_mu_{};                            // posterior mean vector of weights
        MatrixX m_phi_{};                           // [N, M] feature matrix for training
        MatrixX m_phi_sq_{};                        // [N, M] squared feature matrix for training
        MatrixX m_phi_transpose_{};                 // [M, N] transpose of the feature matrix
        MatrixX m_phi_sq_transpose_{};              // [M, N] transpose of the squared feature matrix
        SparseMatrix m_phi_sparse_{};               // [N, M] sparse feature matrix for training
        SparseMatrix m_phi_sq_sparse_{};            // [N, M] sparse squared feature matrix for training
        SparseMatrix m_phi_transpose_sparse_{};     // [M, N] sparse transpose of the feature matrix
        SparseMatrix m_phi_sq_transpose_sparse_{};  // [M, N] sparse transpose of the squared feature matrix
        VectorX m_xi_{};                            // EM xi vector
        VectorX m_lambda_{};                        // EM lambda vector

    public:
        BayesianHilbertMap() = delete;  // delete default constructor

        /**
         * Constructor for BayesianHilbertMap.
         * @param setting settings for the Bayesian Hilbert Map
         * @param kernel kernel function for computing the features
         * @param hinged_points points in the world frame that will be used to compute the Hilbert space features.
         * @param map_boundary the boundary of the map in the world frame. This is used to generate the dataset and to check if a point is inside the map.
         * @param seed random seed for sampling free points. This is to ensure reproducibility of the dataset.
         */
        BayesianHilbertMap(
            std::shared_ptr<BayesianHilbertMapSetting> setting,
            std::shared_ptr<Covariance> kernel,
            MatrixDX hinged_points,
            Aabb<Dtype, Dim> map_boundary,
            uint64_t seed);

        [[nodiscard]] std::shared_ptr<const BayesianHilbertMapSetting>
        GetSetting() const;

        [[nodiscard]] const MatrixDX &
        GetHingedPoints() const;

        [[nodiscard]] const VectorX &
        GetWeights() const;

        [[nodiscard]] const MatrixX &
        GetWeightsCovariance() const;

        [[nodiscard]] const Aabb<Dtype, Dim> &
        GetMapBoundary() const;

        [[nodiscard]] uint64_t
        GetIterationCount() const;

        /**
         * Generate a dataset of {x, y} where x is the position and y is the occupancy label (1 for occupied, 0 for free).
         * @param sensor_position the position of the sensor in the world frame.
         * @param points point cloud in the world frame of the sensor measurement.
         * @param max_dataset_size maximum number of points in the dataset. -1 means no limit.
         * @param num_sample_points number of points in the dataset.
         * @param dataset_points points in the dataset.
         * @param dataset_labels labels of the points in the dataset.
         * @param hit_indices indices of the points that are occupied.
         * @return
         */
        void
        GenerateDataset(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            long max_dataset_size,
            long &num_sample_points,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);

        void
        RunExpectationMaximization(const MatrixDX &points, const VectorX &labels, long num_points);

        void
        PrepareExpectationMaximization(const MatrixDX &points, const VectorX &labels, long num_points);

        void
        RunExpectationMaximizationIteration(long num_points);

        void
        RunExpectationMaximizationIterationSparse(long num_points);

        bool
        Update(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            long &num_points,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);

        /**
         *
         * @param points matrix of points in the world frame. Each column is a point.
         * @param logodd if true, the output will be log-odds instead of probabilities.
         * @param faster if true, the computation will be faster but less accurate.
         * @param compute_gradient if true, the gradient will be computed.
         * @param gradient_with_sigmoid if true, the gradient will be multiplied by the sigmoid function.
         * @param parallel if true, the computation will be parallelized.
         * @param prob_occupied output vector of occupancy probabilities or log-odds.
         * @param gradient output matrix of gradients. If compute_gradient is false, this will not be used.
         */
        void
        Predict(
            const Eigen::Ref<const MatrixDX> &points,
            bool logodd,
            bool faster,
            bool compute_gradient,
            bool gradient_with_sigmoid,
            bool parallel,
            VectorX &prob_occupied,
            MatrixDX &gradient) const;

        void
        PredictSparse(
            const Eigen::Ref<const MatrixDX> &points,
            bool logodd,
            bool faster,
            bool compute_gradient,
            bool gradient_with_sigmoid,
            bool parallel,
            VectorX &prob_occupied,
            MatrixDX &gradient) const;

        void
        PredictGradient(const Eigen::Ref<const MatrixDX> &points, bool faster, bool with_sigmoid, bool parallel, MatrixDX &gradient) const;
    };

}  // namespace erl::geometry

#include "bayesian_hilbert_map.tpp"

template<>
struct YAML::convert<erl::geometry::BayesianHilbertMapSetting> {
    static Node
    encode(const erl::geometry::BayesianHilbertMapSetting &setting) {
        Node node;
        node["diagonal_sigma"] = setting.diagonal_sigma;
        node["max_distance"] = setting.max_distance;
        node["free_points_per_meter"] = setting.free_points_per_meter;
        node["free_sampling_margin"] = setting.free_sampling_margin;
        node["init_sigma"] = setting.init_sigma;
        node["num_em_iterations"] = setting.num_em_iterations;
        node["sparse_zero_threshold"] = setting.sparse_zero_threshold;
        node["use_sparse"] = setting.use_sparse;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::BayesianHilbertMapSetting &setting) {
        if (!node.IsMap()) { return false; }
        setting.diagonal_sigma = node["diagonal_sigma"].as<bool>();
        setting.max_distance = node["max_distance"].as<float>();
        setting.free_points_per_meter = node["free_points_per_meter"].as<float>();
        setting.free_sampling_margin = node["free_sampling_margin"].as<float>();
        setting.init_sigma = node["init_sigma"].as<float>();
        setting.num_em_iterations = node["num_em_iterations"].as<int>();
        setting.sparse_zero_threshold = node["sparse_zero_threshold"].as<float>();
        setting.use_sparse = node["use_sparse"].as<bool>();
        return true;
    }
};
