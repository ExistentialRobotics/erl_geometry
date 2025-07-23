#include "erl_geometry/bayesian_hilbert_map.hpp"

#include "erl_common/block_timer.hpp"
#include "erl_common/random.hpp"
#include "erl_common/serialization.hpp"
#include "erl_geometry/intersection.hpp"

#include <unordered_set>

namespace erl::geometry {

    template<typename Dtype, int Dim>
    BayesianHilbertMap<Dtype, Dim>::BayesianHilbertMap(
        std::shared_ptr<BayesianHilbertMapSetting> setting,
        std::shared_ptr<Covariance> kernel,
        MatrixDX hinged_points,
        Aabb<Dtype, Dim> map_boundary,
        const uint64_t seed)
        : m_setting_(NotNull(std::move(setting), true, "setting is nullptr.")),
          m_kernel_(NotNull(std::move(kernel), true, "kernel is nullptr.")),
          m_hinged_points_(std::move(hinged_points)),
          m_map_boundary_(std::move(map_boundary)),
          m_generator_(seed) {

        const long m = m_hinged_points_.cols() + 1;
        const Dtype sigma = m_setting_->init_sigma;
        const Dtype sigma_inv = 1.0f / sigma;
        if (m_setting_->diagonal_sigma) {
            m_sigma_ = VectorX::Constant(m, sigma);
            m_sigma_inv_ = VectorX::Constant(m, sigma_inv);
        } else {
            m_sigma_ = MatrixX::Zero(m, m);
            m_sigma_inv_ = MatrixX::Zero(m, m);
            for (long i = 0; i < m; ++i) {
                m_sigma_(i, i) = sigma;          // initialize the diagonal of the covariance matrix
                m_sigma_inv_(i, i) = sigma_inv;  // initialize the inverse covariance matrix
            }
        }
        m_mu_ = VectorX::Constant(m, m_setting_->init_mu);
        m_alpha_ = VectorX::Constant(m, m_setting_->init_mu * sigma_inv);
    }

    template<typename Dtype, int Dim>
    std::shared_ptr<const BayesianHilbertMapSetting>
    BayesianHilbertMap<Dtype, Dim>::GetSetting() const {
        return m_setting_;
    }

    template<typename Dtype, int Dim>
    typename BayesianHilbertMap<Dtype, Dim>::MatrixDX &
    BayesianHilbertMap<Dtype, Dim>::GetHingedPoints() {
        return m_hinged_points_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::MatrixDX &
    BayesianHilbertMap<Dtype, Dim>::GetHingedPoints() const {
        return m_hinged_points_;
    }

    template<typename Dtype, int Dim>
    typename BayesianHilbertMap<Dtype, Dim>::VectorX &
    BayesianHilbertMap<Dtype, Dim>::GetWeights() {
        return m_mu_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::VectorX &
    BayesianHilbertMap<Dtype, Dim>::GetWeights() const {
        return m_mu_;
    }

    template<typename Dtype, int Dim>
    typename BayesianHilbertMap<Dtype, Dim>::MatrixX &
    BayesianHilbertMap<Dtype, Dim>::GetWeightsCovariance() {
        return m_sigma_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::MatrixX &
    BayesianHilbertMap<Dtype, Dim>::GetWeightsCovariance() const {
        return m_sigma_;
    }

    template<typename Dtype, int Dim>
    const Aabb<Dtype, Dim> &
    BayesianHilbertMap<Dtype, Dim>::GetMapBoundary() const {
        return m_map_boundary_;
    }

    template<typename Dtype, int Dim>
    uint64_t
    BayesianHilbertMap<Dtype, Dim>::GetIterationCount() const {
        return m_iteration_cnt_;
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::UpdateWithNewWeightCovariance() {
        m_sigma_inv_mat_l_ = m_sigma_.llt().matrixL();
        m_sigma_inv_mat_l_ =
            m_sigma_inv_mat_l_.transpose().template triangularView<Eigen::Upper>().solve(
                MatrixX::Identity(m_sigma_.rows(), m_sigma_.cols()));
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::GenerateDataset(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        long max_dataset_size,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {
        OccupancyMap<Dtype, Dim>::GenerateDataset(
            sensor_position,
            points,
            point_indices,
            m_map_boundary_,
            m_generator_,
            m_setting_->min_distance,
            m_setting_->max_distance,
            m_setting_->free_sampling_margin,
            m_setting_->free_points_per_meter,
            max_dataset_size,
            num_samples,
            dataset_points,
            dataset_labels,
            hit_indices);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximization(
        const MatrixDX &points,
        const VectorX &labels,
        const long num_points) {
        ERL_DEBUG_ASSERT(
            points.cols() == labels.size(),
            "points.cols() = {}, labels.size() = {}.",
            points.cols(),
            labels.size());
        PrepareExpectationMaximization(points, labels, num_points);
        if (m_setting_->use_sparse) {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) {
                RunExpectationMaximizationIterationSparse(num_points);
            }
        } else {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) {
                RunExpectationMaximizationIteration(num_points);
            }
        }
        // release the memory
        if (m_setting_->use_sparse) {
            m_phi_transpose_sparse_ = {};
            m_phi_sq_sparse_ = {};
            m_phi_sq_transpose_sparse_ = {};
        } else {
            m_phi_transpose_ = {};
            m_phi_sq_ = {};
            m_phi_sq_transpose_ = {};
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PrepareExpectationMaximization(
        const MatrixDX &points,
        const VectorX &labels,
        const long num_points) {

        ERL_DEBUG_ASSERT(
            points.cols() >= num_points,
            "points.cols() = {}, num_points = {}.",
            points.cols(),
            num_points);
        const long n_hinged = m_hinged_points_.cols();

        // prepare memory
        if (m_xi_.rows() < num_points) {
            if (!m_setting_->use_sparse) { m_phi_.resize(num_points, n_hinged + 1); }
            m_labels_.resize(num_points);
            m_xi_.resize(num_points);
            m_lambda_.resize(num_points);
        }

        // prepare xi
        m_xi_.head(num_points).setOnes();
        // prepare the labels
        const Dtype *labels_in_ptr = labels.data();
        Dtype *labels_ptr = m_labels_.data();  // labels - 0.5
        for (long j = 0; j < num_points; ++j) { labels_ptr[j] = labels_in_ptr[j] - 0.5f; }

        // prepare phi
        if (m_setting_->use_sparse) {
            m_phi_sparse_.setZero();
            m_phi_sparse_.resize(num_points, n_hinged + 1);  // (N, M + 1)
            m_kernel_->ComputeKtestSparse(
                points,
                num_points,
                m_hinged_points_,
                n_hinged,
                m_setting_->sparse_zero_threshold,
                m_phi_sparse_);
            // add the bias term
            if (m_phi_sparse_.cols() < n_hinged + 1) {
                m_phi_sparse_.conservativeResize(num_points, n_hinged + 1);
            }
            for (long j = 0; j < num_points; ++j) { m_phi_sparse_.insert(j, n_hinged) = 1.0f; }
            m_phi_sparse_.makeCompressed();  // compress the sparse matrix for better performance
            ERL_DEBUG(
                "sparse: {}, dense: {}, sparsity score: {:.3f}",
                m_phi_sparse_.nonZeros(),
                m_phi_sparse_.size(),
                static_cast<Dtype>(m_phi_sparse_.size() - m_phi_sparse_.nonZeros()) /
                    static_cast<Dtype>(m_phi_sparse_.size()));
            m_phi_transpose_sparse_ = m_phi_sparse_.transpose();
            if (m_setting_->diagonal_sigma) {
                m_phi_sq_sparse_ = m_phi_sparse_.cwiseAbs2();
                m_phi_sq_transpose_sparse_ = m_phi_sq_sparse_.transpose();
            }
        } else {
            m_kernel_->ComputeKtest(points, num_points, m_hinged_points_, n_hinged, m_phi_);
            // add the bias term
            m_phi_.template rightCols<1>().setOnes();
            m_phi_transpose_ = m_phi_.transpose();
            if (m_setting_->diagonal_sigma) {
                m_phi_sq_ = m_phi_.cwiseAbs2();
                m_phi_sq_transpose_ = m_phi_sq_.transpose();
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximizationIteration(const long n_points) {
        ++m_iteration_cnt_;

        const long num_feat = m_mu_.rows();

        Dtype *xi_ptr = m_xi_.data();
        Dtype *lam_ptr = m_lambda_.data();
        Dtype *alpha_ptr = m_alpha_.data();

        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        for (long j = 0; j < n_points; ++j) {
            if (xi_ptr[j] < 1.0e-6f) {
                lam_ptr[j] = 0.25f;
            } else {
                lam_ptr[j] = (1.0f / (1.0f + std::exp(-xi_ptr[j])) - 0.5f) / xi_ptr[j];
            }
        }

        if (m_setting_->diagonal_sigma) {  // diagonal sigma
            // E-Step: calculate the posterior
            Dtype *mu_ptr = m_mu_.data();
            Dtype *sigma_inv_ptr = m_sigma_inv_.data();
            Dtype *sigma_ptr = m_sigma_.data();
#pragma omp parallel for default(none) \
    shared(num_feat, n_points, sigma_inv_ptr, alpha_ptr, sigma_ptr, mu_ptr)
            for (long i = 0; i < num_feat; ++i) {  // loop over the features
                sigma_inv_ptr[i] += m_lambda_.head(n_points).dot(m_phi_sq_.col(i).head(n_points));
                alpha_ptr[i] += m_labels_.head(n_points).dot(m_phi_.col(i).head(n_points));
                // sigma_inv' = sigma_inv + 2 * ((phi ** 2).T * lams).sum(dim=1)
                sigma_ptr[i] = 1.0f / sigma_inv_ptr[i];
                // mu' = sigma' * (mu / sigma + phi.T @ (labels - 0.5))
                mu_ptr[i] = sigma_ptr[i] * alpha_ptr[i];
            }
            // M-Step: update xi
#pragma omp parallel for default(none) shared(n_points, xi_ptr)
            for (long j = 0; j < n_points; ++j) {
                Dtype a = m_mu_.dot(m_phi_transpose_.col(j));
                xi_ptr[j] = m_sigma_.col(0).dot(m_phi_sq_transpose_.col(j)) + a * a;
                // xi = sqrt(phi.T @ sigma @ phi + (phi.T @ mu) ** 2)
                xi_ptr[j] = std::sqrt(xi_ptr[j]);
            }
            return;
        }

        // non-diagonal sigma
        // E-Step: calculate the posterior
        // sigma_inv' = sigma_inv + 2 * (phi.T * lams) @ phi
        // mu' = sigma' @ (sigma_inv @ mu + phi.T @ (labels - 0.5))
        // alpha = sigma_inv @ mu
#pragma omp parallel for default(none) shared(num_feat, n_points, alpha_ptr)
        for (long c = 0; c < num_feat; ++c) {  // loop over cols
            auto phi_c = m_phi_.col(c).head(n_points);
            VectorX lam_phi = m_lambda_.head(n_points).cwiseProduct(phi_c);
            alpha_ptr[c] += m_labels_.head(n_points).dot(phi_c);
            Dtype *sigma_inv_ptr = m_sigma_inv_.col(c).data();
            sigma_inv_ptr[c] += lam_phi.dot(phi_c);
            for (long r = c + 1; r < num_feat; ++r) {
                sigma_inv_ptr[r] += lam_phi.dot(m_phi_.col(r).head(n_points));
                // copy the lower triangular part to the upper triangular part
                m_sigma_inv_(c, r) = sigma_inv_ptr[r];
            }
        }
        m_sigma_inv_mat_l_ = m_sigma_inv_.llt().matrixL();  // Cholesky decomposition
        m_mu_ = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(m_alpha_);
        m_sigma_inv_mat_l_.transpose()
            .template triangularView<Eigen::Upper>()  //
            .solveInPlace(m_mu_);
        // M-Step: update xi
#pragma omp parallel for default(none) shared(n_points, num_feat, xi_ptr)
        for (long j = 0; j < n_points; ++j) {
            auto phi_j = m_phi_transpose_.col(j).head(num_feat);
            VectorX phi = phi_j;
            const Dtype a = phi.dot(m_mu_);
            m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solveInPlace(phi);
            xi_ptr[j] = std::sqrt(phi.squaredNorm() + a * a);
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximizationIterationSparse(long num_points) {
        ++m_iteration_cnt_;
        const long num_feat = m_mu_.rows();
        Dtype *xi_ptr = m_xi_.data();
        Dtype *lam_ptr = m_lambda_.data();
        Dtype *alpha_ptr = m_alpha_.data();

        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        for (long j = 0; j < num_points; ++j) {
            if (xi_ptr[j] < 1.0e-6f) {
                lam_ptr[j] = 0.25f;
            } else {
                lam_ptr[j] = (1.0f / (1.0f + std::exp(-xi_ptr[j])) - 0.5f) / xi_ptr[j];
            }
        }

        if (m_setting_->diagonal_sigma) {
            // E-Step: calculate the posterior
            Dtype *mu_ptr = m_mu_.data();
            Dtype *sigma_inv_ptr = m_sigma_inv_.data();
            Dtype *sigma_ptr = m_sigma_.data();
#pragma omp parallel for default(none) \
    shared(num_feat, num_points, sigma_inv_ptr, alpha_ptr, sigma_ptr, mu_ptr)
            for (long i = 0; i < num_feat - 1; ++i) {
                sigma_inv_ptr[i] += m_phi_sq_sparse_.col(i)
                                        .head(num_points)  //
                                        .dot(m_lambda_.head(num_points));
                alpha_ptr[i] += m_phi_sparse_.col(i)
                                    .head(num_points)  //
                                    .dot(m_labels_.head(num_points));
                // sigma_inv' = sigma_inv + 2 * ((phi ** 2).T * lams).sum(dim=1)
                sigma_ptr[i] = 1.0f / sigma_inv_ptr[i];
                // mu' = sigma' * (mu / sigma + phi.T @ (labels - 0.5))
                mu_ptr[i] = sigma_ptr[i] * alpha_ptr[i];
            }
            // last column is the bias term
            const long last_col = num_feat - 1;
            sigma_inv_ptr[last_col] += m_lambda_.head(num_points).sum();
            alpha_ptr[last_col] += m_labels_.head(num_points).sum();
            sigma_ptr[last_col] = 1.0f / sigma_inv_ptr[last_col];
            mu_ptr[last_col] = sigma_ptr[last_col] * alpha_ptr[last_col];
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                Dtype a = m_phi_transpose_sparse_.col(j).dot(m_mu_);
                xi_ptr[j] =
                    std::sqrt(m_phi_sq_transpose_sparse_.col(j).dot(m_sigma_.col(0)) + a * a);
            }
        } else {
            // non-diagonal sigma
            // E-Step: calculate the posterior
#pragma omp parallel for default(none) shared(num_feat, num_points, alpha_ptr)
            for (long c = 0; c < num_feat; ++c) {  // loop over cols
                auto phi_c = m_phi_sparse_.col(c).head(num_points);
                VectorX lam_phi = phi_c.cwiseProduct(m_lambda_.head(num_points));
                alpha_ptr[c] += phi_c.dot(m_labels_.head(num_points));
                Dtype *sigma_inv_ptr = m_sigma_inv_.col(c).data();
                sigma_inv_ptr[c] += phi_c.dot(lam_phi);
                for (long r = c + 1; r < num_feat; ++r) {
                    sigma_inv_ptr[r] += m_phi_sparse_.col(r).head(num_points).dot(lam_phi);
                    // copy the upper triangular part to the lower triangular part
                    m_sigma_inv_(c, r) = sigma_inv_ptr[r];
                }
            }
            m_sigma_inv_mat_l_ = m_sigma_inv_.llt().matrixL();  // Cholesky decomposition
            m_mu_ = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(m_alpha_);
            m_sigma_inv_mat_l_.transpose()
                .template triangularView<Eigen::Upper>()  //
                .solveInPlace(m_mu_);
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, num_feat, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                auto phi_j = m_phi_transpose_sparse_.col(j).head(num_feat);
                const Dtype a = phi_j.dot(m_mu_);
                VectorX phi = phi_j;
                m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solveInPlace(phi);
                xi_ptr[j] = std::sqrt(phi.squaredNorm() + a * a);
            }
        }
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::Update(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        const long max_dataset_size,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {
        GenerateDataset(
            sensor_position,
            points,
            point_indices,
            max_dataset_size,
            num_samples,
            dataset_points,
            dataset_labels,
            hit_indices);
        if (num_samples == 0) {
            ERL_WARN("No valid points generated for update. Skipping update.");
            return false;
        }
        RunExpectationMaximization(dataset_points, dataset_labels, num_samples);
        return true;
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::Predict(
        const Eigen::Ref<const MatrixDX> &points,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        const bool parallel,
        VectorX &prob_occupied,
        MatrixDX &gradient) const {

        // // sparsity does not improve the performance of the prediction
        if (m_setting_->use_sparse) {
            PredictSparse(
                points,
                logodd,
                faster,
                compute_gradient,
                gradient_with_sigmoid,
                parallel,
                prob_occupied,
                gradient);
            return;
        }

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_feats = n_hinged + 1;
        const long n_points = points.cols();
        const long phi_cols = compute_gradient ? n_points * (Dim + 1) : n_points;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_feats, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradient(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                points,
                n_points,
                true,
                phi);
            phi.template bottomRows<1>().head(n_points).setOnes();
            phi.template bottomRows<1>().tail(Dim * n_points).setZero();
            if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        } else {
            m_kernel_->ComputeKtest(m_hinged_points_, n_hinged, points, n_points, phi);
            phi.template bottomRows<1>().head(n_points).setOnes();
        }

        prob_occupied.resize(n_points);
        Dtype *prob_occupied_ptr = prob_occupied.data();
        if (faster) {  // assume sigma is very small; we can use the mean directly
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                const Dtype t1 = phi.col(i).dot(m_mu_);
                prob_occupied_ptr[i] = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);
                Dtype h = t1 / t3;
                prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *grad_phi_x = phi.col(j).data();
                    const Dtype *phi_ptr = phi.col(i).data();
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (long k = 0; k < n_hinged; ++k) {
                        Dtype tmp = mu_ptr[k] - 0.125f * kPI * t1 * sigma_ptr[k] * phi_ptr[k] / t2;
                        grad_ptr[d] += tmp * grad_phi_x[k] / t3;
                    }
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            const Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_  //
                               .template triangularView<Eigen::Lower>()
                               .solve(phi.col(i));
            // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
            const Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
            const Dtype t3 = std::sqrt(t2);
            const Dtype h = t1 / t3;
            prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

            if (!compute_gradient) { continue; }

            // beta = sigma @ phi
            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                Dtype *grad_phi_x = phi.col(j).data();
                for (long k = 0; k < n_hinged; ++k) {
                    const Dtype tmp = 0.125f * kPI * t1 * beta_ptr[k] / t2;
                    grad_ptr[d] += grad_phi_x[k] * (mu_ptr[k] - tmp) / t3;
                }

                if (gradient_with_sigmoid) {
                    const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                    grad_ptr[d] *= p * (1.0f - p);
                }
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::Predict(
        const VectorD &point,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        Dtype &prob_occupied,
        VectorD &gradient) const {

        // // sparsity does not improve the performance of the prediction
        // if (m_setting_->use_sparse) {
        //     PredictSparse(
        //         point,
        //         logodd,
        //         faster,
        //         compute_gradient,
        //         gradient_with_sigmoid,
        //         prob_occupied,
        //         gradient);
        //     return;
        // }

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        const long n_hinged = m_hinged_points_.cols();
        const long phi_cols = compute_gradient ? (Dim + 1) : 1;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Zero(n_hinged);
        MatrixX phi = MatrixX::Zero(n_hinged + 1, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradient(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                point,
                1,
                true,
                phi);
            phi.data()[n_hinged] = 1.0f;  // add the bias term
            // for (int i = 1; i <= Dim; ++i) { phi(n_hinged, i) = 0.0f; }
        } else {
            m_kernel_->ComputeKtest(m_hinged_points_, n_hinged, point, 1, phi);
            phi.data()[n_hinged] = 1.0f;  // add the bias term
        }

        if (faster) {  // assume sigma is very small; we can use the mean directly
            const Dtype t1 = phi.col(0).dot(m_mu_);
            prob_occupied = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

            if (!compute_gradient) { return; }

            for (long d = 0, j = 1; d < Dim; ++d, ++j) {
                gradient[d] = phi.col(j).dot(m_mu_);
                if (gradient_with_sigmoid) {
                    Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied;
                    gradient[d] *= p * (1.0f - p);
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
            Dtype t1 = phi.col(0).dot(m_mu_);
            Dtype t2 = 1.0f + phi.col(0).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);
            Dtype h = t1 / t3;
            prob_occupied = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid

            if (!compute_gradient) { return; }

            for (long d = 0, j = 1; d < Dim; ++d, ++j) {
                const Dtype *grad_phi_x = phi.col(j).data();
                const Dtype *phi_ptr = phi.col(0).data();
                const Dtype *sigma_ptr = m_sigma_.data();
                gradient[d] = 0;
                for (long k = 0; k < n_hinged; ++k) {
                    Dtype tmp = mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi_ptr[k];
                    gradient[d] += tmp * grad_phi_x[k];
                }
                if (gradient_with_sigmoid) {
                    Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied;
                    gradient[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                } else {
                    gradient[d] /= 8.0f * t2 * t3;
                }
            }
            return;
        }

        const Dtype t1 = phi.col(0).dot(m_mu_);
        VectorX beta = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(0));
        // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
        const Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
        const Dtype t3 = std::sqrt(t2);
        const Dtype h = t1 / t3;
        prob_occupied = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

        if (!compute_gradient) { return; }

        // beta = sigma @ phi
        m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
        const Dtype *beta_ptr = beta.data();
        for (long d = 0, j = 1; d < Dim; ++d, ++j) {
            // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
            // phi: n x (n * (Dim + 1))
            gradient[d] = 0;
            Dtype *grad_phi_x = phi.col(j).data();
            for (long k = 0; k < n_hinged; ++k) {
                gradient[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * grad_phi_x[k];
            }

            if (gradient_with_sigmoid) {
                const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied;
                gradient[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
            } else {
                gradient[d] /= 8.0f * t2 * t3;
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PredictSparse(
        const Eigen::Ref<const MatrixDX> &points,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        const bool parallel,
        VectorX &prob_occupied,
        MatrixDX &gradient) const {

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = compute_gradient ? n_points * (Dim + 1) : n_points;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        SparseMatrix phi(n_hinged + 1, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradientSparse(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                points,
                n_points,
                compute_gradient,
                m_setting_->sparse_zero_threshold,
                phi);
            if (phi.rows() < n_hinged + 1) { phi.conservativeResize(n_hinged + 1, phi.cols()); }
            for (long i = 0; i < n_points; ++i) {
                phi.insert(n_hinged, i) = 1.0f;
                for (long d = 0; d < Dim; ++d) {
                    phi.insert(n_hinged, n_points * (d + 1) + i) = 0.0f;
                }
            }
            if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        } else {
            m_kernel_->ComputeKtestSparse(
                m_hinged_points_,
                n_hinged,
                points,
                n_points,
                m_setting_->sparse_zero_threshold,
                phi);
            if (phi.rows() < n_hinged + 1) { phi.conservativeResize(n_hinged + 1, phi.cols()); }
            for (long i = 0; i < n_points; ++i) { phi.insert(n_hinged, i) = 1.0f; }
        }
        phi.makeCompressed();

        prob_occupied.resize(n_points);
        Dtype *prob_occupied_ptr = prob_occupied.data();
        if (faster) {  // assume sigma is very small; we can use the mean directly
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                const Dtype t1 = phi.col(i).dot(m_mu_);
                prob_occupied_ptr[i] = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);
                Dtype h = t1 / t3;
                prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                        const long k = it.row();
                        Dtype tmp = mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi.coeff(k, i);
                        grad_ptr[d] += tmp * it.value();
                    }
                    if (gradient_with_sigmoid) {
                        Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(logodd,                                   \
               n_points,                             \
               phi,                                  \
               prob_occupied_ptr,                    \
               compute_gradient,                     \
               gradient,                             \
               n_hinged,                             \
               mu_ptr,                               \
               gradient_with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_  //
                               .template triangularView<Eigen::Lower>()
                               .solve(phi.col(i).toDense());
            // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
            Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);
            Dtype h = t1 / t3;
            prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

            if (!compute_gradient) { continue; }

            // beta = sigma @ phi
            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                    const long k = it.row();
                    grad_ptr[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * it.value();
                }
                if (gradient_with_sigmoid) {
                    const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                    grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                } else {
                    grad_ptr[d] /= 8.0f * t2 * t3;
                }
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PredictSparse(
        const VectorD &point,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        Dtype &prob_occupied,
        VectorD &gradient) const {

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        const long n_hinged = m_hinged_points_.cols();
        const long phi_cols = compute_gradient ? (Dim + 1) : 1;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        SparseMatrix phi(n_hinged + 1, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradientSparse(
                m_hinged_points_,
                n_hinged,
                grad_flags,
                point,
                1,
                true,
                m_setting_->sparse_zero_threshold,
                phi);
            if (phi.rows() < n_hinged + 1) { phi.conservativeResize(n_hinged + 1, phi_cols); }
            phi.insert(n_hinged, 0) = 1.0f;  // add the bias term
            for (long d = 0; d < Dim; ++d) { phi.insert(n_hinged, d + 1) = 0.0f; }
        } else {
            m_kernel_->ComputeKtestSparse(
                m_hinged_points_,
                n_hinged,
                point,
                1,
                m_setting_->sparse_zero_threshold,
                phi);
            if (phi.rows() < n_hinged + 1) { phi.conservativeResize(n_hinged + 1, phi_cols); }
            phi.insert(n_hinged, 0) = 1.0f;
        }
        phi.makeCompressed();

        if (faster) {  // assume sigma is very small; we can use the mean directly
            const Dtype t1 = phi.col(0).dot(m_mu_);
            prob_occupied = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

            if (!compute_gradient) { return; }

            for (long d = 0, j = 1; d < Dim; ++d, ++j) {
                gradient[d] = phi.col(j).dot(m_mu_);
                if (gradient_with_sigmoid) {
                    Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied;
                    gradient[d] *= p * (1.0f - p);
                }
            }

            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
            Dtype t1 = phi.col(0).dot(m_mu_);
            Dtype t2 = 1.0f + phi.col(0).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);
            Dtype h = t1 / t3;
            prob_occupied = logodd ? h : 1.0f / (1.0f + std::exp(-h));

            if (!compute_gradient) { return; }

            for (long d = 0, j = 1; d < Dim; ++d, ++j) {
                const Dtype *sigma_ptr = m_sigma_.data();
                gradient[d] = 0;
                for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                    const long k = it.row();
                    Dtype tmp = mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi.coeff(k, 0);
                    gradient[d] += tmp * it.value();
                }
                if (gradient_with_sigmoid) {
                    Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied;
                    gradient[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                } else {
                    gradient[d] /= 8.0f * t2 * t3;
                }
            }
            return;
        }

        Dtype t1 = phi.col(0).dot(m_mu_);
        VectorX beta = m_sigma_inv_mat_l_  //
                           .template triangularView<Eigen::Lower>()
                           .solve(phi.col(0).toDense());
        // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
        Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
        Dtype t3 = std::sqrt(t2);
        Dtype h = t1 / t3;
        prob_occupied = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

        if (!compute_gradient) { return; }

        // beta = sigma @ phi
        m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
        const Dtype *beta_ptr = beta.data();
        for (long d = 0, j = 1; d < Dim; ++d, ++j) {
            // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
            // phi: n x (n * (Dim + 1))
            gradient[d] = 0;
            for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                const long k = it.row();
                gradient[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * it.value();
            }
            if (gradient_with_sigmoid) {
                const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied;
                gradient[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
            } else {
                gradient[d] /= 8.0f * t2 * t3;
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::PredictGradient(
        const Eigen::Ref<const MatrixDX> &points,
        const bool faster,
        const bool with_sigmoid,
        const bool parallel,
        MatrixDX &gradient) const {

        constexpr auto kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = n_points * (Dim + 1);

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_hinged + 1, phi_cols);
        m_kernel_->ComputeKtestWithGradient(
            m_hinged_points_,
            n_hinged,
            grad_flags,
            points,
            n_points,
            true,
            phi);
        phi.template bottomRows<1>().head(n_points).setOnes();
        phi.template bottomRows<1>().tail(Dim * n_points).setZero();
        if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }

        if (faster) {  // assume sigma is very small, we can use the mean directly
#pragma omp parallel for if (parallel) default(none) shared(n_points, phi, gradient, with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (with_sigmoid) {
                        Dtype t1 = phi.col(i).dot(m_mu_);
                        Dtype y = 1.0f / (1.0f + std::exp(-t1));
                        grad_ptr[d] *= y * (1.0f - y);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {
            // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(n_points, phi, gradient, n_hinged, mu_ptr, with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *grad_phi_x = phi.col(j).data();
                    const Dtype *phi_ptr = phi.col(i).data();
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (long k = 0; k < n_hinged; ++k) {
                        Dtype tmp = mu_ptr[k] * t3 - kPI * t1 * sigma_ptr[k] * phi_ptr[k];
                        grad_ptr[d] += tmp * grad_phi_x[k];
                    }
                    if (with_sigmoid) {
                        Dtype h = t1 / t3;
                        Dtype y = 1.0f / (1.0f + std::exp(-h));  // sigmoid function
                        grad_ptr[d] *= y * (1.0f - y) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(n_points, phi, gradient, n_hinged, mu_ptr, with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta =
                m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(i));
            // 1 + phi^T @ sigma @ phi * (M_PI / 8.0)
            Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);
            Dtype t3 = std::sqrt(t2);

            // beta = sigma @ phi
            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                Dtype *grad_phi_x = phi.col(j).data();
                for (long k = 0; k < n_hinged; ++k) {
                    grad_ptr[d] += (mu_ptr[k] * t3 - M_PI * t1 * beta_ptr[k]) * grad_phi_x[k];
                }
                if (with_sigmoid) {
                    Dtype h = t1 / t3;
                    Dtype y = 1.0f / (1.0f + std::exp(-h));  // sigmoid function
                    grad_ptr[d] *= y * (1.0f - y) / (8.0f * t2 * t3);
                } else {
                    grad_ptr[d] /= 8.0f * t2 * t3;
                }
            }
        }
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::Write(std::ostream &s) const {
        using namespace common;
        static const TokenWriteFunctionPairs<BayesianHilbertMap> token_function_pairs = {
            {
                "setting",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return self->m_setting_->Write(stream) && stream.good();
                },
            },
            {
                "kernel",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return self->m_kernel_->Write(stream) && stream.good();
                },
            },
            {
                "hinged_points",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_hinged_points_) &&
                           stream.good();
                },
            },
            {
                "map_boundary",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return stream.write(
                               reinterpret_cast<const char *>(self->m_map_boundary_.center.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.write(
                               reinterpret_cast<const char *>(
                                   self->m_map_boundary_.half_sizes.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.good();
                },
            },
            {
                "generator",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    stream << self->m_generator_ << '\n';
                    return stream.good();
                },
            },
            {
                "iteration_cnt",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    stream.write(
                        reinterpret_cast<const char *>(&self->m_iteration_cnt_),
                        sizeof(uint64_t));
                    return stream.good();
                },
            },
            {
                "sigma_inv",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_sigma_inv_) &&
                           stream.good();
                },
            },
            {
                "sigma",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_sigma_) && stream.good();
                },
            },
            {
                "sigma_inv_mat_l",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_sigma_inv_mat_l_) &&
                           stream.good();
                },
            },
            {
                "alpha",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_alpha_) && stream.good();
                },
            },
            {
                "labels",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_labels_) && stream.good();
                },
            },
            {
                "mu",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_mu_) && stream.good();
                },
            },
            {
                "phi",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_) && stream.good();
                },
            },
            {
                "phi_sq",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_sq_) && stream.good();
                },
            },
            {
                "phi_transpose",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_phi_sq_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(stream, self->m_phi_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(stream, self->m_phi_sq_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_transpose_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(
                               stream,
                               self->m_phi_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose_sparse",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveSparseEigenMatrixToBinaryStream(
                               stream,
                               self->m_phi_sq_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "xi",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_xi_) && stream.good();
                },
            },
            {
                "lambda",
                [](const BayesianHilbertMap *self, std::ostream &stream) {
                    return SaveEigenMatrixToBinaryStream(stream, self->m_lambda_) && stream.good();
                },
            },
        };
        return WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::Read(std::istream &s) {
        using namespace common;
        static const TokenReadFunctionPairs<BayesianHilbertMap> token_function_pairs = {
            {
                "setting",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return self->m_setting_->Read(stream) && stream.good();
                },
            },
            {
                "kernel",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return self->m_kernel_->Read(stream) && stream.good();
                },
            },
            {
                "hinged_points",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_hinged_points_) &&
                           stream.good();
                },
            },
            {
                "map_boundary",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return stream.read(
                               reinterpret_cast<char *>(self->m_map_boundary_.center.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.read(
                               reinterpret_cast<char *>(self->m_map_boundary_.half_sizes.data()),
                               sizeof(Dtype) * Dim) &&
                           stream.good();
                },
            },
            {
                "generator",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    stream >> self->m_generator_;
                    SkipLine(stream);
                    return stream.good();
                },
            },
            {
                "iteration_cnt",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    stream.read(
                        reinterpret_cast<char *>(&self->m_iteration_cnt_),
                        sizeof(uint64_t));
                    return stream.good();
                },
            },
            {
                "sigma_inv",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_sigma_inv_) &&
                           stream.good();
                },
            },
            {
                "sigma",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_sigma_) && stream.good();
                },
            },
            {
                "sigma_inv_mat_l",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_sigma_inv_mat_l_) &&
                           stream.good();
                },
            },
            {
                "alpha",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_alpha_) && stream.good();
                },
            },
            {
                "labels",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_labels_) &&
                           stream.good();
                },
            },
            {
                "mu",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_mu_) && stream.good();
                },
            },
            {
                "phi",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_) && stream.good();
                },
            },
            {
                "phi_sq",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_sq_) &&
                           stream.good();
                },
            },
            {
                "phi_transpose",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_phi_sq_transpose_) &&
                           stream.good();
                },
            },
            {
                "phi_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(stream, self->m_phi_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(stream, self->m_phi_sq_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_transpose_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(
                               stream,
                               self->m_phi_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "phi_sq_transpose_sparse",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadSparseEigenMatrixFromBinaryStream(
                               stream,
                               self->m_phi_sq_transpose_sparse_) &&
                           stream.good();
                },
            },
            {
                "xi",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_xi_) && stream.good();
                },
            },
            {
                "lambda",
                [](BayesianHilbertMap *self, std::istream &stream) {
                    return LoadEigenMatrixFromBinaryStream(stream, self->m_lambda_) &&
                           stream.good();
                },
            },
        };
        return ReadTokens(s, this, token_function_pairs);
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::operator==(const BayesianHilbertMap &other) const {
        if (m_setting_ == nullptr && other.m_setting_ != nullptr) { return false; }
        if (m_setting_ != nullptr &&
            (other.m_setting_ == nullptr || *m_setting_ != *other.m_setting_)) {
            return false;
        }
        if (m_kernel_ == nullptr && other.m_kernel_ != nullptr) { return false; }
        if (m_kernel_ != nullptr &&
            (other.m_kernel_ == nullptr || *m_kernel_ != *other.m_kernel_)) {
            return false;
        }
        using namespace common;
        if (!SafeEigenMatrixEqual(m_hinged_points_, other.m_hinged_points_)) { return false; }
        if (m_map_boundary_ != other.m_map_boundary_) { return false; }
        // if (m_generator_ != other.m_generator_) { return false; }  // cannot compare
        if (m_iteration_cnt_ != other.m_iteration_cnt_) { return false; }
        if (!SafeEigenMatrixEqual(m_sigma_inv_, other.m_sigma_inv_)) { return false; }
        if (!SafeEigenMatrixEqual(m_sigma_, other.m_sigma_)) { return false; }
        if (!SafeEigenMatrixEqual(m_sigma_inv_mat_l_, other.m_sigma_inv_mat_l_)) { return false; }
        if (!SafeEigenMatrixEqual(m_alpha_, other.m_alpha_)) { return false; }
        if (!SafeEigenMatrixEqual(m_labels_, other.m_labels_)) { return false; }
        if (!SafeEigenMatrixEqual(m_mu_, other.m_mu_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_, other.m_phi_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_sq_, other.m_phi_sq_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_transpose_, other.m_phi_transpose_)) { return false; }
        if (!SafeEigenMatrixEqual(m_phi_sq_transpose_, other.m_phi_sq_transpose_)) { return false; }
        for (auto &[left, right]:
             std::vector<std::pair<const SparseMatrix &, const SparseMatrix &>>{
                 {m_phi_sparse_, other.m_phi_sparse_},
                 {m_phi_sq_sparse_, other.m_phi_sq_sparse_},
                 {m_phi_transpose_sparse_, other.m_phi_transpose_sparse_},
                 {m_phi_sq_transpose_sparse_, other.m_phi_sq_transpose_sparse_},
             }) {
            if (!SafeSparseEigenMatrixEqual(left, right)) { return false; }
        }
        if (!SafeEigenMatrixEqual(m_xi_, other.m_xi_)) { return false; }
        if (!SafeEigenMatrixEqual(m_lambda_, other.m_lambda_)) { return false; }
        return true;
    }

    template<typename Dtype, int Dim>
    bool
    BayesianHilbertMap<Dtype, Dim>::operator!=(const BayesianHilbertMap &other) const {
        return !(*this == other);
    }

    template class BayesianHilbertMap<float, 2>;
    template class BayesianHilbertMap<double, 2>;
    template class BayesianHilbertMap<float, 3>;
    template class BayesianHilbertMap<double, 3>;
}  // namespace erl::geometry

YAML::Node
YAML::convert<erl::geometry::BayesianHilbertMapSetting>::encode(
    const erl::geometry::BayesianHilbertMapSetting &setting) {
    Node node;
    ERL_YAML_SAVE_ATTR(node, setting, diagonal_sigma);
    ERL_YAML_SAVE_ATTR(node, setting, min_distance);
    ERL_YAML_SAVE_ATTR(node, setting, max_distance);
    ERL_YAML_SAVE_ATTR(node, setting, free_points_per_meter);
    ERL_YAML_SAVE_ATTR(node, setting, free_sampling_margin);
    ERL_YAML_SAVE_ATTR(node, setting, init_mu);
    ERL_YAML_SAVE_ATTR(node, setting, init_sigma);
    ERL_YAML_SAVE_ATTR(node, setting, num_em_iterations);
    ERL_YAML_SAVE_ATTR(node, setting, sparse_zero_threshold);
    ERL_YAML_SAVE_ATTR(node, setting, use_sparse);
    return node;
}

bool
YAML::convert<erl::geometry::BayesianHilbertMapSetting>::decode(
    const Node &node,
    erl::geometry::BayesianHilbertMapSetting &setting) {
    if (!node.IsMap()) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, diagonal_sigma);
    ERL_YAML_LOAD_ATTR(node, setting, min_distance);
    ERL_YAML_LOAD_ATTR(node, setting, max_distance);
    ERL_YAML_LOAD_ATTR(node, setting, free_points_per_meter);
    ERL_YAML_LOAD_ATTR(node, setting, free_sampling_margin);
    ERL_YAML_LOAD_ATTR(node, setting, init_mu);
    ERL_YAML_LOAD_ATTR(node, setting, init_sigma);
    ERL_YAML_LOAD_ATTR(node, setting, num_em_iterations);
    ERL_YAML_LOAD_ATTR(node, setting, sparse_zero_threshold);
    ERL_YAML_LOAD_ATTR(node, setting, use_sparse);
    return true;
}
