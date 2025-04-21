#pragma once

#include "erl_geometry/intersection.hpp"

namespace erl::geometry {

    template<typename Dtype, int Dim>
    BayesianHilbertMap<Dtype, Dim>::BayesianHilbertMap(
        std::shared_ptr<BayesianHilbertMapSetting> setting,
        std::shared_ptr<Covariance> kernel,
        MatrixDX hinged_points,
        Aabb<Dtype, Dim> map_boundary,
        const uint64_t seed)
        : m_setting_(std::move(setting)),
          m_kernel_(std::move(kernel)),
          m_hinged_points_(std::move(hinged_points)),
          m_map_boundary_(std::move(map_boundary)),
          m_generator_(seed) {

        const long m = m_hinged_points_.cols();
        const Dtype sigma = m_setting_->init_sigma;
        const Dtype sigma_inv = 1.0f / sigma;
        if (m_setting_->diagonal_sigma) {
            m_sigma_ = VectorX::Zero(m);
            m_sigma_inv_ = VectorX::Zero(m);
            for (long i = 0; i < m; ++i) {
                m_sigma_(i, 0) = sigma;          // initialize the diagonal of the covariance matrix
                m_sigma_inv_(i, 0) = sigma_inv;  // initialize the inverse covariance matrix
            }
        } else {
            m_sigma_ = MatrixX::Zero(m, m);
            m_sigma_inv_ = MatrixX::Zero(m, m);
            for (long i = 0; i < m; ++i) {
                m_sigma_(i, i) = sigma;          // initialize the diagonal of the covariance matrix
                m_sigma_inv_(i, i) = sigma_inv;  // initialize the inverse covariance matrix
            }
        }
        m_mu_ = VectorX::Zero(m);
        m_alpha_ = VectorX::Zero(m);
    }

    template<typename Dtype, int Dim>
    std::shared_ptr<const BayesianHilbertMapSetting>
    BayesianHilbertMap<Dtype, Dim>::GetSetting() const {
        return m_setting_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::MatrixDX &
    BayesianHilbertMap<Dtype, Dim>::GetHingedPoints() const {
        return m_hinged_points_;
    }

    template<typename Dtype, int Dim>
    const typename BayesianHilbertMap<Dtype, Dim>::VectorX &
    BayesianHilbertMap<Dtype, Dim>::GetWeights() const {
        return m_mu_;
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
    BayesianHilbertMap<Dtype, Dim>::GenerateDataset(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const long max_dataset_size,
        long &num_sample_points,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {

        // 1. check if the ray intersects with the map boundary.
        // 2. compute the range to sample free points and the number of points to sample.
        // 3. sample the free points uniformly within the range.
        // 4. return the result.

        const Dtype max_distance = m_setting_->max_distance;
        const Dtype free_points_per_meter = m_setting_->free_points_per_meter;
        const Dtype free_sampling_margin = m_setting_->free_sampling_margin;

        std::vector<std::tuple<long, bool, long, Dtype, Dtype>> infos;  // tuple of (point_index, hit_flag, num_free_points, d1, d2)
        infos.reserve(points.cols() / 10);                              // reserve space for the infos
        long num_total_free_points = 0;
        long num_total_hit_points = 0;
        hit_indices.clear();

        for (long i = 0; i < points.cols(); ++i) {
            VectorD point = points.col(i);
            VectorD v = point - sensor_position;
            Dtype v_norm = v.norm();
            v /= v_norm;  // normalize the vector
            Dtype d1 = 0;
            Dtype d2 = 0;
            bool hit_flag = false;
            bool intersected = false;
            bool is_inside = false;
            // compute intersection between the ray (point -> sensor_position) and the map boundary
            geometry::ComputeIntersectionBetweenRayAndAabb<Dtype, Dim>(
                sensor_position,
                v.cwiseInverse(),
                m_map_boundary_.min(),
                m_map_boundary_.max(),
                d1,
                d2,
                intersected,
                is_inside);

            if (!intersected ||             // the ray does not intersect with the map boundary
                (d1 < 0 && d2 < 0) ||       // the ray hits a point outside the map, v points away from the map
                (v_norm <= d1 && d1 <= d2)  // the ray hits a point outside the map, v points toward the map
            ) {
                continue;
            }
            hit_flag = m_map_boundary_.contains(point) && (v_norm < max_distance);  // check if the point is inside the map
            if (is_inside) {                                                        // the ray hits a point inside the map, d2 < 0 is useless
                d1 = free_sampling_margin * v_norm;
                d2 = std::min((1.0f - free_sampling_margin) * v_norm, d1);
            } else {
                d1 = std::max(free_sampling_margin * v_norm, d1);
                d2 = std::min((1.0f - free_sampling_margin) * v_norm, d2);
            }
            auto n = std::max(0l, static_cast<long>(std::ceil((d2 - d1) * free_points_per_meter)));  // number of free points to sample
            if (n == 0 && !hit_flag) { continue; }                                                   // no free points to sample and the point is not hit
            num_total_free_points += n;                                                              // count the number of free points to sample
            num_total_hit_points += static_cast<long>(hit_flag);                                     // count the number of hit points
            d1 /= v_norm;
            d2 /= v_norm;
            infos.emplace_back(i, hit_flag, n, d1, d2);
        }

        ERL_DEBUG_ASSERT(
            (num_total_hit_points == 0 && num_total_free_points == 0) || (num_total_hit_points > 0 && num_total_free_points > 0),
            "num_total_hit_points = {}, num_total_free_points = {}.",
            num_total_hit_points,
            num_total_free_points);

        long num_sample_hit_points = num_total_hit_points;
        long num_sample_free_points = num_total_free_points;
        num_sample_points = num_sample_hit_points + num_sample_free_points;
        const bool size_limit_exceeded = max_dataset_size > 0 && num_sample_points > max_dataset_size;
        if (size_limit_exceeded) {
            num_sample_hit_points = max_dataset_size * num_sample_hit_points / num_sample_points;
            num_sample_free_points = max_dataset_size * num_sample_free_points / num_sample_points;
            num_sample_points = num_sample_hit_points + num_sample_free_points;
            ERL_DEBUG_ASSERT(num_sample_points <= max_dataset_size, "num_sample_points = {}, max_dataset_size = {}.", num_sample_points, max_dataset_size);
            std::shuffle(infos.begin(), infos.end(), m_generator_);  // shuffle the infos to sample uniformly
            infos.resize(num_sample_hit_points);
        }
        if (dataset_points.cols() < num_sample_points) {
            dataset_points.resize(Dim, num_sample_points);
            dataset_labels.resize(num_sample_points);
        }
        Dtype *points_ptr = dataset_points.data();
        Dtype *labels_ptr = dataset_labels.data();
        num_sample_points = 0;  // reset the number of points to 0
        for (const auto &[point_index, hit_flag, num_free_points, d1, d2]: infos) {
            if (max_dataset_size > 0 && num_sample_points >= max_dataset_size) { break; }
            const Dtype *point_ptr = points.col(point_index).data();
            if (hit_flag) {
                std::memcpy(points_ptr, point_ptr, sizeof(Dtype) * Dim);  // copy the hit point to the result
                *labels_ptr++ = 1.0f;                                     // label as occupied and move the pointer to the next position
                points_ptr += Dim;                                        // move the pointer to the next position
                ++num_sample_points;                                      // increment the number of points
                hit_indices.push_back(point_index);                       // add the index to the list
            }
            if (num_total_free_points == 0) { continue; }  // no free points to sample.
            // sample points in free space uniformly within the range [d1, d2]
            long n = num_free_points;
            if (size_limit_exceeded) { n = std::min(num_free_points * num_sample_free_points / num_total_free_points, max_dataset_size - num_sample_points); }
            if (n <= 0) { continue; }  // no free points to sample
            num_sample_points += n;

            std::uniform_real_distribution<Dtype> distribution(d1, d2);
            for (long j = 0; j < n; ++j) {
                Dtype r = distribution(m_generator_);  // sample a random distance within the range [d1, d2]
                Dtype s = 1 - r;
                for (long k = 0; k < Dim; ++k, ++points_ptr) { *points_ptr = sensor_position[k] * s + point_ptr[k] * r; }  // compute the free point position
                *labels_ptr++ = 0.0f;                                                                                      // label as free
            }
        }
        ERL_DEBUG_ASSERT(
            max_dataset_size < 0 || num_sample_points <= max_dataset_size,
            "num_sample_points = {}, max_dataset_size = {}.",
            num_sample_points,
            max_dataset_size);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximization(const MatrixDX &points, const VectorX &labels, const long num_points) {
        ERL_DEBUG_ASSERT(points.cols() == labels.size(), "points.cols() = {}, labels.size() = {}.", points.cols(), labels.size());
        PrepareExpectationMaximization(points, labels, num_points);
        if (m_setting_->use_sparse) {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) { RunExpectationMaximizationIterationSparse(num_points); }
        } else {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) { RunExpectationMaximizationIteration(num_points); }
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
    BayesianHilbertMap<Dtype, Dim>::PrepareExpectationMaximization(const MatrixDX &points, const VectorX &labels, const long num_points) {

        // ERL_BLOCK_TIMER_MSG("bhm prepare expectation maximization");

        ERL_DEBUG_ASSERT(points.cols() >= num_points, "points.cols() = {}, num_points = {}.", points.cols(), num_points);

        // prepare memory
        if (m_phi_.rows() < num_points) {
            m_phi_.resize(num_points, m_hinged_points_.cols());
            m_labels_.resize(num_points);
            m_xi_.resize(num_points);
            m_lambda_.resize(num_points);
        }

        // prepare xi
        m_xi_.head(num_points).setOnes();
        // prepare the labels
        const Dtype *labels_in_ptr = labels.data();
        Dtype *labels_ptr = m_labels_.data();
        for (long j = 0; j < num_points; ++j) { labels_ptr[j] = labels_in_ptr[j] - 0.5f; }  // labels - 0.5

        // prepare phi
        if (m_setting_->use_sparse) {
            m_kernel_->ComputeKtestSparse(points, num_points, m_hinged_points_, m_hinged_points_.cols(), m_setting_->sparse_zero_threshold, m_phi_sparse_);
            ERL_DEBUG(
                "sparse: {}, dense: {}, sparsity score: {:.3f}",
                m_phi_sparse_.nonZeros(),
                m_phi_sparse_.size(),
                static_cast<Dtype>(m_phi_sparse_.size() - m_phi_sparse_.nonZeros()) / static_cast<Dtype>(m_phi_sparse_.size()));
            m_phi_transpose_sparse_ = m_phi_sparse_.transpose();
            if (m_setting_->diagonal_sigma) {
                m_phi_sq_sparse_ = m_phi_sparse_.cwiseAbs2();
                m_phi_sq_transpose_sparse_ = m_phi_sq_sparse_.transpose();
            }
        } else {
            m_kernel_->ComputeKtest(points, num_points, m_hinged_points_, m_hinged_points_.cols(), m_phi_);
            m_phi_transpose_ = m_phi_.transpose();
            if (m_setting_->diagonal_sigma) {
                m_phi_sq_ = m_phi_.cwiseAbs2();
                m_phi_sq_transpose_ = m_phi_sq_.transpose();
            }
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximizationIteration(const long num_points) {

        // ERL_BLOCK_TIMER_MSG("bhm run expectation maximization iteration");

        ERL_TRACY_FRAME_MARK_START();

        ++m_iteration_cnt_;

        const long num_feat = m_hinged_points_.cols();

        Dtype *xi_ptr = m_xi_.data();
        Dtype *lam_ptr = m_lambda_.data();
        Dtype *alpha_ptr = m_alpha_.data();

        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        for (long j = 0; j < num_points; ++j) { lam_ptr[j] = (1.0f / (1.0f + std::exp(-xi_ptr[j])) - 0.5f) / xi_ptr[j]; }

        if (m_setting_->diagonal_sigma) {  // diagonal sigma
            // E-Step: calculate the posterior
            Dtype *mu_ptr = m_mu_.data();
            Dtype *sigma_inv_ptr = m_sigma_inv_.data();
            Dtype *sigma_ptr = m_sigma_.data();
#pragma omp parallel for default(none) shared(num_feat, num_points, sigma_inv_ptr, alpha_ptr, sigma_ptr, mu_ptr)
            for (long i = 0; i < num_feat; ++i) {  // loop over the features
                sigma_inv_ptr[i] += m_lambda_.head(num_points).dot(m_phi_sq_.col(i).head(num_points));
                alpha_ptr[i] += m_labels_.head(num_points).dot(m_phi_.col(i).head(num_points));
                sigma_ptr[i] = 1.0f / sigma_inv_ptr[i];   // sigma_inv' = sigma_inv + 2 * ((phi ** 2).T * lams).sum(dim=1)
                mu_ptr[i] = sigma_ptr[i] * alpha_ptr[i];  // mu' = sigma' * (mu / sigma + phi.T @ (labels - 0.5))
            }
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                Dtype a = m_mu_.dot(m_phi_transpose_.col(j));
                xi_ptr[j] = m_sigma_.col(0).dot(m_phi_sq_transpose_.col(j)) + a * a;
                xi_ptr[j] = std::sqrt(xi_ptr[j]);  // xi = sqrt(phi.T @ sigma @ phi + (phi.T @ mu) ** 2)
            }
        } else {
            // non-diagonal sigma
            // E-Step: calculate the posterior
            // sigma_inv' = sigma_inv + 2 * (phi.T * lams) @ phi
            // mu' = sigma' @ (sigma_inv @ mu + phi.T @ (labels - 0.5))
            // alpha = sigma_inv @ mu
#pragma omp parallel for default(none) shared(num_feat, num_points, alpha_ptr)
            for (long c = 0; c < num_feat; ++c) {  // loop over cols
                auto phi_c = m_phi_.col(c).head(num_points);
                VectorX lam_phi = m_lambda_.head(num_points).cwiseProduct(phi_c);
                alpha_ptr[c] += m_labels_.head(num_points).dot(phi_c);
                Dtype *sigma_inv_ptr = m_sigma_inv_.col(c).data();
                sigma_inv_ptr[c] += lam_phi.dot(phi_c);
                for (long r = c + 1; r < num_feat; ++r) {
                    sigma_inv_ptr[r] += lam_phi.dot(m_phi_.col(r).head(num_points));
                    m_sigma_inv_(c, r) = sigma_inv_ptr[r];  // copy the lower triangular part to the upper triangular part
                }
            }
            m_sigma_inv_mat_l_ = m_sigma_inv_.llt().matrixL();  // Cholesky decomposition
            m_mu_ = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(m_alpha_);
            m_sigma_inv_mat_l_.transpose().template triangularView<Eigen::Upper>().solveInPlace(m_mu_);
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, num_feat, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                auto phi_j = m_phi_transpose_.col(j).head(num_feat);
                const Dtype a = phi_j.dot(m_mu_);
                VectorX phi = phi_j;
                m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solveInPlace(phi);
                xi_ptr[j] = std::sqrt(phi.squaredNorm() + a * a);
            }
        }

        ERL_TRACY_FRAME_MARK_END();
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMap<Dtype, Dim>::RunExpectationMaximizationIterationSparse(long num_points) {
        ++m_iteration_cnt_;
        const long num_feat = m_hinged_points_.cols();
        Dtype *xi_ptr = m_xi_.data();
        Dtype *lam_ptr = m_lambda_.data();
        Dtype *alpha_ptr = m_alpha_.data();

        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        for (long j = 0; j < num_points; ++j) { lam_ptr[j] = (1.0f / (1.0f + std::exp(-xi_ptr[j])) - 0.5f) / xi_ptr[j]; }

        if (m_setting_->diagonal_sigma) {
            // E-Step: calculate the posterior
            Dtype *mu_ptr = m_mu_.data();
            Dtype *sigma_inv_ptr = m_sigma_inv_.data();
            Dtype *sigma_ptr = m_sigma_.data();
#pragma omp parallel for default(none) shared(num_feat, num_points, sigma_inv_ptr, alpha_ptr, sigma_ptr, mu_ptr)
            for (long i = 0; i < num_feat; ++i) {
                sigma_inv_ptr[i] += m_phi_sq_sparse_.col(i).head(num_points).dot(m_lambda_.head(num_points));
                alpha_ptr[i] += m_phi_sparse_.col(i).head(num_points).dot(m_labels_.head(num_points));
                sigma_ptr[i] = 1.0f / sigma_inv_ptr[i];   // sigma_inv' = sigma_inv + 2 * ((phi ** 2).T * lams).sum(dim=1)
                mu_ptr[i] = sigma_ptr[i] * alpha_ptr[i];  // mu' = sigma' * (mu / sigma + phi.T @ (labels - 0.5))
            }
            // M-Step: update xi
#pragma omp parallel for default(none) shared(num_points, xi_ptr)
            for (long j = 0; j < num_points; ++j) {
                Dtype a = m_phi_transpose_sparse_.col(j).dot(m_mu_);
                xi_ptr[j] = std::sqrt(m_phi_sq_transpose_sparse_.col(j).dot(m_sigma_.col(0)) + a * a);
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
                    m_sigma_inv_(c, r) = sigma_inv_ptr[r];  // copy the upper triangular part to the lower triangular part
                }
            }
            m_sigma_inv_mat_l_ = m_sigma_inv_.llt().matrixL();  // Cholesky decomposition
            m_mu_ = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(m_alpha_);
            m_sigma_inv_mat_l_.transpose().template triangularView<Eigen::Upper>().solveInPlace(m_mu_);
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
        long &num_points,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {
        GenerateDataset(sensor_position, points, num_points, dataset_points, dataset_labels, hit_indices);
        if (num_points == 0) {
            ERL_WARN("No valid points generated for update. Skipping update.");
            return false;
        }
        RunExpectationMaximization(dataset_points, dataset_labels, num_points);
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

        // sparsity does not improve the performance of the prediction
        // if (m_setting_->use_sparse) {
        //     PredictSparse(points, logodd, faster, compute_gradient, gradient_with_sigmoid, parallel, prob_occupied, gradient);
        //     return;
        // }

        constexpr Dtype kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = compute_gradient ? n_points * (Dim + 1) : n_points;

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_hinged, phi_cols);
        if (compute_gradient) {
            m_kernel_->ComputeKtestWithGradient(m_hinged_points_, n_hinged, grad_flags, points, n_points, true, phi);
            if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        } else {
            m_kernel_->ComputeKtest(m_hinged_points_, n_hinged, points, n_points, phi);
        }

        prob_occupied.resize(n_points);
        Dtype *prob_occupied_ptr = prob_occupied.data();
        if (faster) {  // assume sigma is very small; we can use the mean directly
#pragma omp parallel for if (parallel) default(none) shared(logodd, n_points, phi, prob_occupied_ptr, compute_gradient, gradient, gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                const Dtype t1 = phi.col(i).dot(m_mu_);
                prob_occupied_ptr[i] = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (gradient_with_sigmoid) {
                        const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {  // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd, n_points, phi, prob_occupied_ptr, compute_gradient, gradient, n_hinged, mu_ptr, gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);
                Dtype h = t1 / t3;
                prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *grad_phi_x = phi.col(j).data();
                    const Dtype *phi_ptr = phi.col(i).data();
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (long k = 0; k < n_hinged; ++k) { grad_ptr[d] += (mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi_ptr[k]) * grad_phi_x[k]; }
                    if (gradient_with_sigmoid) {
                        const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(logodd, n_points, phi, prob_occupied_ptr, compute_gradient, gradient, n_hinged, mu_ptr, gradient_with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            const Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(i));
            const Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);  // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
            const Dtype t3 = std::sqrt(t2);
            const Dtype h = t1 / t3;
            prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

            if (!compute_gradient) { continue; }

            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);  // beta = sigma @ phi
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                Dtype *grad_phi_x = phi.col(j).data();
                for (long k = 0; k < n_hinged; ++k) { grad_ptr[d] += (mu_ptr[k] * t2 - kPI * t1 * beta_ptr[k]) * grad_phi_x[k]; }

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
        SparseMatrix phi(n_hinged, phi_cols);
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
            if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        } else {
            m_kernel_->ComputeKtestSparse(m_hinged_points_, n_hinged, points, n_points, m_setting_->sparse_zero_threshold, phi);
        }

        prob_occupied.resize(n_points);
        Dtype *prob_occupied_ptr = prob_occupied.data();
        if (faster) {  // assume sigma is very small; we can use the mean directly
#pragma omp parallel for if (parallel) default(none) shared(logodd, n_points, phi, prob_occupied_ptr, compute_gradient, gradient, gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                const Dtype t1 = phi.col(i).dot(m_mu_);
                prob_occupied_ptr[i] = logodd ? t1 : 1.0f / (1.0f + std::exp(-t1));

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    grad_ptr[d] = phi.col(j).dot(m_mu_);
                    if (gradient_with_sigmoid) {
                        const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-t1)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p);
                    }
                }
            }
            return;
        }

        const Dtype *mu_ptr = m_mu_.data();
        if (m_setting_->diagonal_sigma) {  // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) \
    shared(logodd, n_points, phi, prob_occupied_ptr, compute_gradient, gradient, n_hinged, mu_ptr, gradient_with_sigmoid)
            for (long i = 0; i < n_points; ++i) {
                Dtype t1 = phi.col(i).dot(m_mu_);
                Dtype t2 = 1.0f + phi.col(i).cwiseAbs2().dot(m_sigma_.col(0)) * (kPI / 8.0f);
                Dtype t3 = std::sqrt(t2);
                Dtype h = t1 / t3;
                prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

                if (!compute_gradient) { continue; }

                Dtype *grad_ptr = gradient.col(i).data();
                for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                    const Dtype *sigma_ptr = m_sigma_.data();
                    grad_ptr[d] = 0;
                    for (typename SparseMatrix::InnerIterator it(phi, j); it; ++it) {
                        const long k = it.row();
                        grad_ptr[d] += (mu_ptr[k] * t2 - kPI * t1 * sigma_ptr[k] * phi.coeff(k, i)) * it.value();
                    }
                    if (gradient_with_sigmoid) {
                        const Dtype p = logodd ? 1.0f / (1.0f + std::exp(-h)) : prob_occupied_ptr[i];
                        grad_ptr[d] *= p * (1.0f - p) / (8.0f * t2 * t3);
                    } else {
                        grad_ptr[d] /= 8.0f * t2 * t3;
                    }
                }
            }
            return;
        }

#pragma omp parallel for if (parallel) default(none) \
    shared(logodd, n_points, phi, prob_occupied_ptr, compute_gradient, gradient, n_hinged, mu_ptr, gradient_with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(i).toDense());
            Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);  // 1 + phi^T @ sigma @ phi * (kPI / 8.0)
            Dtype t3 = std::sqrt(t2);
            Dtype h = t1 / t3;
            prob_occupied_ptr[i] = logodd ? h : 1.0f / (1.0f + std::exp(-h));  // sigmoid function

            if (!compute_gradient) { continue; }

            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);  // beta = sigma @ phi
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
    BayesianHilbertMap<Dtype, Dim>::PredictGradient(
        const Eigen::Ref<const MatrixDX> &points,
        const bool faster,
        const bool with_sigmoid,
        const bool parallel,
        MatrixDX &gradient) const {

        constexpr Dtype kPI = static_cast<Dtype>(M_PI);
        (void) parallel;
        const long n_hinged = m_hinged_points_.cols();
        const long n_points = points.cols();
        const long phi_cols = n_points * (Dim + 1);

        const Eigen::VectorXl grad_flags = Eigen::VectorXl::Constant(n_hinged, 0);
        MatrixX phi(n_hinged, phi_cols);
        m_kernel_->ComputeKtestWithGradient(m_hinged_points_, n_hinged, grad_flags, points, n_points, true, phi);
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
        if (m_setting_->diagonal_sigma) {  // assume sigma is diagonal, i.e. element of mu is independent
#pragma omp parallel for if (parallel) default(none) shared(n_points, phi, gradient, n_hinged, mu_ptr, with_sigmoid)
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
                    for (long k = 0; k < n_hinged; ++k) { grad_ptr[d] += (mu_ptr[k] * t3 - kPI * t1 * sigma_ptr[k] * phi_ptr[k]) * grad_phi_x[k]; }
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

#pragma omp parallel for if (parallel) default(none) shared(n_points, phi, gradient, n_hinged, mu_ptr, with_sigmoid)
        for (long i = 0; i < n_points; ++i) {
            Dtype t1 = phi.col(i).dot(m_mu_);
            VectorX beta = m_sigma_inv_mat_l_.template triangularView<Eigen::Lower>().solve(phi.col(i));
            Dtype t2 = 1.0f + beta.squaredNorm() * (kPI / 8.0f);  // 1 + phi^T @ sigma @ phi * (M_PI / 8.0)
            Dtype t3 = std::sqrt(t2);

            m_sigma_inv_mat_l_.template triangularView<Eigen::Upper>().solveInPlace(beta);  // beta = sigma @ phi
            Dtype *grad_ptr = gradient.col(i).data();
            const Dtype *beta_ptr = beta.data();
            for (long d = 0, j = n_points + i; d < Dim; ++d, j += n_points) {
                // grad = grad_phi_x @ grad_phi_h @ grad_prob_h
                // phi: n x (n * (Dim + 1))
                grad_ptr[d] = 0;
                Dtype *grad_phi_x = phi.col(j).data();
                for (long k = 0; k < n_hinged; ++k) { grad_ptr[d] += (mu_ptr[k] * t3 - M_PI * t1 * beta_ptr[k]) * grad_phi_x[k]; }
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

}  // namespace erl::geometry
