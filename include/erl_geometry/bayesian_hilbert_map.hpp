#pragma once

#include "aabb.hpp"

#include "erl_common/random.hpp"
#include "erl_common/yaml.hpp"
#include "erl_covariance/covariance.hpp"

namespace erl::geometry {

    struct BayesianHilbertMapSetting : common::Yamlable<BayesianHilbertMapSetting> {
        // if true, the covariance matrix will be assumed to be diagonal to speed up the
        // computation.
        bool diagonal_sigma = false;
        // maximum distance from the sensor to consider a point as occupied.
        float max_distance = 30.0f;
        // number of free points to sample per meter from the sensor.
        float free_points_per_meter = 2.0f;
        // percentage margin to use when sampling free points to avoid sampling too close to the
        // surface or the sensor.
        float free_sampling_margin = 0.05f;
        // initial value for initializing the mean vector. 0.0f means unknown. <0.0f means we
        // assume the point is occupied, >0.0f means we assume the point is free.
        float init_mu = 0.0f;
        // initial value for initializing the covariance matrix.
        float init_sigma = 1.0e4f;
        // number of iterations for the Expectation-Maximization (EM) algorithm to optimize the mean
        // and covariance.
        int num_em_iterations = 3;
        // threshold for sparse matrix zeroing.
        float sparse_zero_threshold = 1e-6f;
        // if true, use sparse matrix for the feature matrix.
        bool use_sparse = false;
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
        // settings for the Bayesian Hilbert Map
        std::shared_ptr<BayesianHilbertMapSetting> m_setting_ = nullptr;
        std::shared_ptr<Covariance> m_kernel_ = nullptr;
        // [Dim, N] hinged points for computing the hilbert space features
        MatrixDX m_hinged_points_{};
        Aabb<Dtype, Dim> m_map_boundary_{};
        std::mt19937_64 m_generator_;
        // iteration count for the EM algorithm (just for statistics)
        uint64_t m_iteration_cnt_ = 0;
        // inverse of the posterior weight covariance
        MatrixX m_sigma_inv_{};
        // posterior covariance of the weights
        MatrixX m_sigma_{};
        // matrix L for Cholesky decomposition of m_sigma_inv_, used when diagonal_sigma is false
        MatrixX m_sigma_inv_mat_l_{};
        // alpha = sigma_inv * mu
        VectorX m_alpha_{};
        // label = input_label - 0.5
        VectorX m_labels_{};
        // posterior mean vector of weights
        VectorX m_mu_{};
        // [N, M] feature matrix for training
        MatrixX m_phi_{};
        // [N, M] squared feature matrix for training
        MatrixX m_phi_sq_{};
        // [M, N] transpose of the feature matrix
        MatrixX m_phi_transpose_{};
        // [M, N] transpose of the squared feature matrix
        MatrixX m_phi_sq_transpose_{};
        // [N, M] sparse feature matrix for training
        SparseMatrix m_phi_sparse_{};
        // [N, M] sparse squared feature matrix for training
        SparseMatrix m_phi_sq_sparse_{};
        // [M, N] sparse transpose of the feature matrix
        SparseMatrix m_phi_transpose_sparse_{};
        // [M, N] sparse transpose of the squared feature matrix
        SparseMatrix m_phi_sq_transpose_sparse_{};
        // EM xi vector
        VectorX m_xi_{};
        // EM lambda vector
        VectorX m_lambda_{};

    public:
        BayesianHilbertMap() = delete;  // delete default constructor

        /**
         * Constructor for BayesianHilbertMap.
         * @param setting settings for the Bayesian Hilbert Map
         * @param kernel kernel function for computing the features
         * @param hinged_points points in the world frame that will be used to compute the Hilbert
         * space features.
         * @param map_boundary The boundary of the map in the world frame. This is used to generate
         * the dataset and to check if a point is inside the map.
         * @param seed Random seed for sampling free points. This is to ensure reproducibility of
         * the dataset.
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
         * Generate a dataset of {x, y} where x is the position and y is the occupancy label (1 for
         * occupied, 0 for free).
         * @param sensor_position the position of the sensor in the world frame.
         * @param points point cloud in the world frame of the sensor measurement.
         * @param point_indices indices of the points in the point cloud that are valid for dataset.
         * If empty, all points will be used.
         * @param max_dataset_size maximum number of points in the dataset. -1 means no limit.
         * @param num_samples number of points in the dataset.
         * @param dataset_points points in the dataset.
         * @param dataset_labels labels of the points in the dataset.
         * @param hit_indices indices of the points that are occupied.
         * @return
         */
        void
        GenerateDataset(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<long> &point_indices,
            long max_dataset_size,
            long &num_samples,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);

        void
        RunExpectationMaximization(const MatrixDX &points, const VectorX &labels, long num_points);

        void
        PrepareExpectationMaximization(
            const MatrixDX &points,
            const VectorX &labels,
            long num_points);

        void
        RunExpectationMaximizationIteration(long n_points);

        void
        RunExpectationMaximizationIterationSparse(long num_points);

        bool
        Update(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            long max_dataset_size,
            long &num_points,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);

        /**
         *
         * @param points Matrix of points in the world frame. Each column is a point.
         * @param logodd If true, the output will be log-odds instead of probabilities.
         * @param faster If true, the computation will be faster but less accurate.
         * @param compute_gradient If true, the gradient will be computed.
         * @param gradient_with_sigmoid If true, the gradient will be multiplied by the sigmoid
         * function.
         * @param parallel If true, the computation will be parallelized.
         * @param prob_occupied Output vector of occupancy probabilities or log-odds.
         * @param gradient Output matrix of gradients. If compute_gradient is false, this will not
         * be used.
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
        Predict(
            const VectorD &point,
            bool logodd,
            bool faster,
            bool compute_gradient,
            bool gradient_with_sigmoid,
            Dtype &prob_occupied,
            VectorD &gradient) const;

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
        PredictGradient(
            const Eigen::Ref<const MatrixDX> &points,
            bool faster,
            bool with_sigmoid,
            bool parallel,
            MatrixDX &gradient) const;

        [[nodiscard]] bool
        Write(std::ostream &s) const;

        [[nodiscard]] bool
        Read(std::istream &s);

        [[nodiscard]] bool
        operator==(const BayesianHilbertMap &other) const;

        [[nodiscard]] bool
        operator!=(const BayesianHilbertMap &other) const;

    private:
        void
        GenerateRayInfos(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<long> &point_indices,
            std::vector<std::tuple<long, bool, long, Dtype, Dtype>> &infos,
            long &max_num_free_points,
            long &max_num_hit_points);

        void
        GenerateSamples(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<std::tuple<long, bool, long, Dtype, Dtype>> &infos,
            bool random_infos,
            long num_hit_to_sample,
            long num_free_to_sample,
            long &num_samples,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);
    };

}  // namespace erl::geometry

#include "bayesian_hilbert_map.tpp"

template<>
struct YAML::convert<erl::geometry::BayesianHilbertMapSetting> {
    static Node
    encode(const erl::geometry::BayesianHilbertMapSetting &setting);

    static bool
    decode(const Node &node, erl::geometry::BayesianHilbertMapSetting &setting);
};
