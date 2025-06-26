#ifdef ERL_USE_LIBTORCH

    #include "erl_geometry/bayesian_hilbert_map_torch.hpp"

namespace erl::geometry {
    template<typename Dtype, int Dim>
    BayesianHilbertMapTorch<Dtype, Dim>::BayesianHilbertMapTorch(
        std::shared_ptr<geometry::BayesianHilbertMapSetting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::LoadHingedPoints(const MatrixDX &hinged_points) {
        m_hinged_points_ = torch::from_blob(
            const_cast<Dtype *>(hinged_points.data()),
            {1, hinged_points.cols(), Dim},
            torch::TensorOptions().dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64));
        m_hinged_points_ = m_hinged_points_.clone();
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::PrepareMemory(const long num_maps, const long num_points) {
        m_num_maps_ = num_maps;
        m_num_points_ = num_points;
        ERL_DEBUG_ASSERT(
            m_num_maps_ > 0 && m_num_points_ > 0,
            "Number of maps and points must be greater than 0.");

        m_num_loaded_points_.resize(m_num_maps_, 0);
        m_weights_loaded_.resize(m_num_maps_, false);

        ERL_ASSERTM(m_hinged_points_.defined(), "Hinged points are not loaded.");
        const long num_features = m_hinged_points_.size(1) + 1;

        // stay on CPU for now, as we need to load the dataset first, map weights and covariance
        // before moving to GPU.
        if (!m_sigma_.defined() || m_sigma_.size(0) != m_num_maps_ ||
            m_sigma_.size(1) != num_features) {
            if (m_setting_->diagonal_sigma) {
                m_sigma_ = torch::zeros(
                    {m_num_maps_, num_features, 1},
                    torch::TensorOptions()
                        .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                        .device(torch::kCPU));
            } else {
                m_sigma_ = torch::zeros(
                    {m_num_maps_, num_features, num_features},
                    torch::TensorOptions()
                        .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                        .device(torch::kCPU));
            }
        }
        if (!m_mu_.defined() || m_mu_.size(0) != m_num_maps_ || m_mu_.size(1) != num_features) {
            m_mu_ = torch::zeros(
                {m_num_maps_, num_features, 1},
                torch::TensorOptions()
                    .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                    .device(torch::kCPU));
        }
        if (!m_map_centers_cpu_.defined() || m_map_centers_cpu_.size(0) != m_num_maps_) {
            m_map_centers_cpu_ = torch::zeros(
                {m_num_maps_, 1, Dim},
                torch::TensorOptions()
                    .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                    .device(torch::kCPU));
        }
        if (!m_points_cpu_.defined() || m_points_cpu_.size(0) != m_num_maps_ ||
            m_points_cpu_.size(1) != m_num_points_) {
            m_points_cpu_ = torch::zeros(
                {m_num_maps_, m_num_points_, Dim},
                torch::TensorOptions()
                    .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                    .device(torch::kCPU));
        }
        if (!m_labels_cpu_.defined() || m_labels_cpu_.size(0) != m_num_maps_ ||
            m_labels_cpu_.size(1) != m_num_points_) {
            m_labels_cpu_ = torch::zeros(
                {m_num_maps_, m_num_points_, 1},
                torch::TensorOptions()
                    .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                    .device(torch::kCPU));
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::LoadWeightsAndCovariance(
        const long map_idx,
        const VectorX &weights,
        const MatrixX &cov) {
        ERL_DEBUG_ASSERT(
            weights.size() == m_mu_.size(1),
            "weights.size() = {}, m_mu_.size(1) = {}.",
            weights.size(),
            m_mu_.size(1));
        ERL_DEBUG_ASSERT(
            cov.rows() == m_sigma_.size(1) && cov.cols() == m_sigma_.size(2),
            "cov shape: {}x{}, m_sigma_ shape: {}x{}.",
            cov.rows(),
            cov.cols(),
            m_sigma_.size(1),
            m_sigma_.size(2));
        auto *ptr = m_mu_.mutable_data_ptr<Dtype>();
        ptr += map_idx * m_mu_.size(1);
        std::memcpy(ptr, weights.data(), sizeof(Dtype) * m_mu_.size(1));

        const long sigma_size = m_sigma_.size(1) * m_sigma_.size(2);
        ptr = m_sigma_.mutable_data_ptr<Dtype>();
        ptr += map_idx * sigma_size;  // could be (M, M) or (M, ).
        std::memcpy(ptr, cov.data(), sizeof(Dtype) * sigma_size);

        m_weights_loaded_[map_idx] = true;
        m_weights_changed_ = true;
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::GetWeightsAndCovariance(
        long map_idx,
        VectorX &weights,
        MatrixX &cov) {
        ERL_DEBUG_ASSERT(
            weights.size() == m_mu_.size(1),
            "weights.size() = {}, m_mu_.size(1) = {}.",
            weights.size(),
            m_mu_.size(1));
        ERL_DEBUG_ASSERT(
            cov.rows() == m_sigma_.size(1) && cov.cols() == m_sigma_.size(2),
            "cov shape: {}x{}, m_sigma_ shape: {}x{}.",
            cov.rows(),
            cov.cols(),
            m_sigma_.size(1),
            m_sigma_.size(2));
        if (!m_mu_.defined() || !m_sigma_.defined() || map_idx < 0 || map_idx >= m_num_maps_) {
            ERL_WARN("Weights and covariance are unavailable for map index {}.", map_idx);
            return;
        }

        torch::cuda::synchronize();  // ensure all previous operations are done

        if (m_mu_.device() != torch::kCPU) {
            torch::Tensor mu = m_mu_.index({map_idx}).to(torch::kCPU);
            torch::Tensor sigma = m_sigma_.index({map_idx}).to(torch::kCPU);
            auto *ptr = mu.data_ptr<Dtype>();
            std::memcpy(weights.data(), ptr, sizeof(Dtype) * weights.size());
            ptr = sigma.data_ptr<Dtype>();
            std::memcpy(cov.data(), ptr, sizeof(Dtype) * cov.size());
            return;
        }

        auto *ptr = m_mu_.data_ptr<Dtype>();
        ptr += map_idx * m_mu_.size(1);
        std::memcpy(weights.data(), ptr, sizeof(Dtype) * weights.size());

        const long sigma_size = m_sigma_.size(1) * m_sigma_.size(2);
        ptr = m_sigma_.data_ptr<Dtype>();
        ptr += map_idx * sigma_size;  // could be (M + 1, M + 1) or (M + 1, ).
        std::memcpy(cov.data(), ptr, sizeof(Dtype) * sigma_size);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::LoadDataset(
        const long map_idx,
        const VectorD &map_center,
        const MatrixDX &points,
        const VectorX &labels,
        const long num_points) {

        auto *ptr = m_map_centers_cpu_.mutable_data_ptr<Dtype>();
        ptr += map_idx * Dim;
        std::memcpy(ptr, map_center.data(), sizeof(Dtype) * Dim);

        ptr = m_points_cpu_.mutable_data_ptr<Dtype>();
        ptr += map_idx * m_points_cpu_.size(1) * Dim;
        std::memcpy(ptr, points.data(), sizeof(Dtype) * num_points * Dim);

        ptr = m_labels_cpu_.mutable_data_ptr<Dtype>();
        ptr += map_idx * m_labels_cpu_.size(1);
        std::memcpy(ptr, labels.data(), sizeof(Dtype) * num_points);

        m_num_loaded_points_[map_idx] = num_points;
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::RunExpectationMaximization() {
        PrepareExpectationMaximization();
        if (m_setting_->use_sparse) {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) {
                RunExpectationMaximizationIterationSparse();
            }
        } else {
            for (int itr = 0; itr < m_setting_->num_em_iterations; ++itr) {
                RunExpectationMaximizationIteration();
            }
        }
        FinishExpectationMaximization();
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::PrepareExpectationMaximization() {

        ERL_DEBUG_ASSERT(
            m_hinged_points_.defined(),
            "Hinged points are not loaded, call LoadHingedPoints() first.");
        ERL_DEBUG_ASSERT(m_points_cpu_.defined(), "call PrepareMemory() first.");
        ERL_DEBUG_ASSERT(
            std::any_of(
                m_weights_loaded_.begin(),
                m_weights_loaded_.end(),
                [](bool loaded) { return loaded; }),
            "Weights and covariance are not loaded, call LoadWeightsAndCovariance() first.");

        // move to GPU
        m_hinged_points_ = m_hinged_points_.to(m_device_);
        m_sigma_ = m_sigma_.to(m_device_);
        m_mu_ = m_mu_.to(m_device_);
        m_map_centers_ = m_map_centers_cpu_.to(m_device_);
        m_points_ = m_points_cpu_.to(m_device_);
        m_labels_ = m_labels_cpu_.to(m_device_);
        m_labels_ -= 0.5f;  // convert labels from {0, 1} to {-0.5, 0.5}

        m_xi_ = torch::ones(
            {m_num_maps_, m_num_points_, 1},
            torch::TensorOptions()
                .dtype(sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64)
                .device(m_device_));

        if (m_weights_changed_) {
            // m_sigma_inv_ is not the inverse yet
            // m_alpha_ is not sigma_inv @ mu yet
            if (m_setting_->diagonal_sigma) {
                m_sigma_inv_ = m_sigma_.reciprocal();
                m_alpha_ = m_sigma_inv_ * m_mu_;
            } else {
                m_sigma_inv_ = torch::linalg_cholesky(m_sigma_).cholesky_inverse();  // (B,M+1,M+1)
                m_alpha_ = m_sigma_inv_.matmul(m_mu_);                               // (B,M+1,1)
            }
            m_weights_changed_ = false;
        } else {
            m_sigma_inv_ = m_sigma_inv_.to(m_device_);
            m_alpha_ = m_alpha_.to(m_device_);
        }

        // compute the feature matrix
        // |a-b|^2 = |a|^2 + |b|^2 - 2 * a^T b
        // (1, M, Dim) -> (1, M)
        const torch::Tensor sq_norm_a = m_hinged_points_.square().sum(2);
        // (B, N, Dim) - (B, 1, Dim) -> (B, N, Dim) -> (B, N)
        torch::Tensor diff = m_points_ - m_map_centers_;
        const torch::Tensor sq_norm_b = diff.square().sum(2);
        // (1, M, 1) + (B, 1, N) -> (B, M, N)
        m_phi_ = sq_norm_a.unsqueeze(2) + sq_norm_b.unsqueeze(1) -
                 2 * torch::einsum("ij,klj->kil", {m_hinged_points_.squeeze(0), diff});
        // add the bias term, put it in the last row. put zeros so that they become ones later.
        // (B, M, N) -> (B, M + 1, N)
        m_phi_ = torch::cat(
            {
                m_phi_,
                torch::zeros({m_num_maps_, 1, m_num_points_}, m_phi_.options()),
            },
            1);
        const Dtype gamma = -0.5f / (m_kernel_scale_ * m_kernel_scale_);
        if (m_setting_->use_sparse) {
            const Dtype threshold = std::log(m_setting_->sparse_zero_threshold) / gamma;
            for (long b = 0; b < m_num_maps_; ++b) {
                if (m_num_loaded_points_[b] >= m_num_points_) { continue; }
                (void) m_phi_.index({b}).index_fill_(
                    1,
                    torch::arange(m_num_loaded_points_[b], m_num_points_).to(m_device_),
                    threshold);
            }
            const torch::Tensor mask = m_phi_ < threshold;
            // convert phi from shape (B, M+1, N) to (B * (M + 1), B * N), a block diagonal matrix.
            m_phi_ = ToBlockDiagonal(
                mask.nonzero().t(),
                torch::exp(m_phi_.masked_select(mask) * gamma),
                m_num_maps_,
                m_phi_.size(1),
                m_phi_.size(2));
        } else {
            m_phi_ = torch::exp(m_phi_ * gamma);
            // unused features are set to 0
            for (long b = 0; b < m_num_maps_; ++b) {
                (void) m_phi_.index({b}).index_fill_(
                    1,
                    torch::arange(m_num_loaded_points_[b], m_num_points_).to(m_device_),
                    0);
            }
        }
        if (m_setting_->diagonal_sigma) { m_phi_sq_ = m_phi_.square(); }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::RunExpectationMaximizationIteration() {
        constexpr Dtype kEpsilon = 1.0e-10f;
        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        // m_lambda_ = (torch::sigmoid(m_xi_) - 0.5f) / m_xi_;  // (B, N, 1)
        m_lambda_ = torch::where(m_xi_ > kEpsilon, (torch::sigmoid(m_xi_) - 0.5f) / m_xi_, 0.25f);
        if (m_setting_->diagonal_sigma) {
            // E-step: calculate the posterior
            m_sigma_inv_ += m_phi_sq_.matmul(m_lambda_);  // (B, M+1, N) @ (B, N, 1) -> (B, M+1, 1)
            m_alpha_ += m_phi_.matmul(m_labels_);         // (B, M+1, N) @ (B, N, 1) -> (B, M+1, 1)
            m_sigma_ = m_sigma_inv_.reciprocal();
            m_mu_ = m_sigma_ * m_alpha_;
            // M-step: update xi
            // (B, 1, M+1) @ (B, M+1, N) -> (B, 1, N) -> (B, N) -> (B, N, 1)
            m_xi_ = (m_mu_.squeeze(2).unsqueeze(1).matmul(m_phi_).square().squeeze(1) +
                     m_sigma_.squeeze(2).unsqueeze(1).matmul(m_phi_sq_).squeeze(1))
                        .sqrt()
                        .unsqueeze(2);
        } else {
            // non-diagonal sigma
            // E-step: calculate the posterior
            // (B, M+1, N) * (B, 1, N) -> (B, M+1, N)
            torch::Tensor lambda_phi = m_phi_.mul(m_lambda_.squeeze(2).unsqueeze(1));
            m_sigma_inv_ += torch::einsum("ijk,ilk->ijl", {lambda_phi, m_phi_});
            m_alpha_ += m_phi_.matmul(m_labels_);  // (B, M+1, N) @ (B, N, 1) -> (B, M+1, 1)
            m_sigma_ = torch::linalg_cholesky(m_sigma_inv_).cholesky_inverse();
            m_mu_ = m_sigma_.matmul(m_alpha_);  // (B, M+1, M+1) @ (B, M+1, 1) -> (B, M+1, 1)
            // M-step: update xi
            // (B, 1, M+1) @ (B, M+1, N) -> (B, 1, N) -> (B, N) -> (B, N, 1)
            // (B, M+1, M+1) @ (B, M+1, N) -> (B, M+1, N) -> (B, N) -> (B, N, 1)
            m_xi_ = (m_mu_.squeeze(2).unsqueeze(1).matmul(m_phi_).square().squeeze(1) +
                     m_sigma_.matmul(m_phi_).mul(m_phi_).sum(1))
                        .sqrt()
                        .unsqueeze(2);
        }

        ERL_DEBUG_ASSERT(!m_lambda_.isnan().any().item<bool>(), "NaN detected in lambda.");
        ERL_DEBUG_ASSERT(!m_sigma_inv_.isnan().any().item<bool>(), "NaN detected in sigma_inv.");
        ERL_DEBUG_ASSERT(!m_alpha_.isnan().any().item<bool>(), "NaN detected in alpha.");
        ERL_DEBUG_ASSERT(!m_sigma_.isnan().any().item<bool>(), "NaN detected in sigma.");
        ERL_DEBUG_ASSERT(!m_mu_.isnan().any().item<bool>(), "NaN detected in mu.");
        ERL_DEBUG_ASSERT(!m_xi_.isnan().any().item<bool>(), "NaN detected in xi.");
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::RunExpectationMaximizationIterationSparse() {
        constexpr Dtype kEpsilon = 1.0e-10f;
        // calculate 2 * lambda(xi) = (sigmoid(xi) - 0.5) / xi
        // m_lambda_ = (torch::sigmoid(m_xi_) - 0.5f) / m_xi_;  // (B, N, 1)
        m_lambda_ = torch::where(m_xi_ > kEpsilon, (torch::sigmoid(m_xi_) - 0.5f) / m_xi_, 0.25f);
        if (m_setting_->diagonal_sigma) {
            // E-step: calculate the posterior
            // (B(M+1), BN) @ (BN, 1) -> (B(M+1), 1) -> (B, M+1, 1)
            m_sigma_inv_ += m_phi_sq_.matmul(m_lambda_.view({-1, 1})).view({m_num_maps_, -1, 1});
            m_alpha_ += m_phi_.matmul(m_labels_.view({-1, 1})).view({m_num_maps_, -1, 1});
            m_sigma_ = m_sigma_inv_.reciprocal();
            m_mu_ = m_sigma_ * m_alpha_;
            // (1, B(M+1)) @ (B(M+1), BN) -> (1, BN) -> (B, N, 1)
            torch::Tensor t1 = m_mu_.view({1, -1}).matmul(m_phi_).view({m_num_maps_, -1, 1});
            torch::Tensor t2 = m_sigma_.view({1, -1}).matmul(m_phi_sq_).view({m_num_maps_, -1, 1});
            m_xi_ = (t1.square() + t2).sqrt();  // (B, N, 1)
        } else {
            // non-diagonal sigma
            // E-step: calculate the posterior
            // (B(M+1), BN) * (1, BN) -> (BM, BN), sparse
            const long kB = m_num_maps_;
            const long kF = m_hinged_points_.size(1) + 1;
            const long kN = m_num_points_;
            torch::Tensor lambda_phi = m_phi_.mul(m_lambda_.view({1, -1}));
            torch::Tensor sigma_delta = lambda_phi.matmul(m_phi_.t());  // (B(M+1), B(M+1))
            sigma_delta = ToBatched(sigma_delta.indices(), sigma_delta.values(), kB, kF, kF);
            // m_sigma_inv_ += sigma_delta;  // batched inplace sparse addition is not supported yet
            m_sigma_inv_ = m_sigma_inv_ + sigma_delta;
            // (B(M+1), BN) @ (BN, 1) -> (B(M+1), 1) -> (B, (M+1), 1)
            m_alpha_ += m_phi_.matmul(m_labels_.view({kB * kN, 1})).view({kB, kF, 1});
            m_sigma_ = torch::linalg_cholesky(m_sigma_inv_).cholesky_inverse().contiguous();
            m_mu_ = m_sigma_.matmul(m_alpha_);  // (B, M+1, M+1) @ (B, M+1, 1) -> (B, M+1, 1)
            // M-step: update xi
            // (B, 1, M+1) @ (B, M+1, N) -> (B, 1, N) -> (B, N) -> (B, N, 1)
            torch::Tensor t1 = m_mu_.view({1, kB * kF}).matmul(m_phi_).view({kB, kN, 1});
            // (BN, B(M+1)) @ (B(M+1), M) -> (BN, M+1)
            torch::Tensor t2 = m_phi_.t().matmul(m_sigma_.view({kB * kF, kF})).contiguous();
            t2 = t2.mul_(CollapseDim(m_phi_.indices(), m_phi_.values(), kB, kF, kN, 0).t())
                     .sum(1)
                     .view({kB, kN, 1});
            m_xi_ = (t1.square() + t2).sqrt();  // (B, N, 1)
        }
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::FinishExpectationMaximization() {
        torch::cuda::synchronize();
        m_mu_ = m_mu_.to(torch::kCPU);
        m_sigma_ = m_sigma_.to(torch::kCPU);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::ToCpu() {
        m_hinged_points_ = m_hinged_points_.to(torch::kCPU);
        m_sigma_inv_ = m_sigma_inv_.to(torch::kCPU);
        m_sigma_ = m_sigma_.to(torch::kCPU);
        m_alpha_ = m_alpha_.to(torch::kCPU);
        m_mu_ = m_mu_.to(torch::kCPU);
        m_map_centers_ = m_map_centers_.to(torch::kCPU);
        m_points_ = m_points_.to(torch::kCPU);
        m_labels_ = m_labels_.to(torch::kCPU);
        m_phi_ = m_phi_.to(torch::kCPU);
        m_xi_ = m_xi_.to(torch::kCPU);
        m_lambda_ = m_lambda_.to(torch::kCPU);
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::Reset() {
        m_hinged_points_.reset();
        m_sigma_inv_.reset();
        m_sigma_.reset();
        m_alpha_.reset();
        m_mu_.reset();
        m_map_centers_.reset();
        m_map_centers_cpu_.reset();
        m_points_.reset();
        m_points_cpu_.reset();
        m_labels_.reset();
        m_labels_cpu_.reset();
        m_phi_.reset();
        m_phi_sq_.reset();
        m_xi_.reset();
        m_lambda_.reset();
    }

    template<typename Dtype, int Dim>
    void
    BayesianHilbertMapTorch<Dtype, Dim>::Predict(
        const long map_idx,
        const Eigen::Ref<const MatrixDX> &points,
        const bool logodd,
        const bool faster,
        const bool compute_gradient,
        const bool gradient_with_sigmoid,
        VectorX &prob_occupied,
        MatrixDX &gradient) const {

        const long n_points = points.cols();  // N
        torch::Tensor points_tensor =         // (N, Dim)
            torch::from_blob(
                const_cast<Dtype *>(points.data()),
                {n_points, Dim},
                torch::TensorOptions().dtype(
                    sizeof(Dtype) == 4 ? torch::kFloat32 : torch::kFloat64))
                .clone()
                .to(m_device_);

        torch::Tensor logodd_tensor, t1, t2, t3;
        const torch::Tensor mu = m_mu_.index({map_idx});        // (M+1, 1)
        const torch::Tensor sigma = m_sigma_.index({map_idx});  // (M+1, M+1) or (M+1, 1)
        const Dtype gamma = 0.5f / (m_kernel_scale_ * m_kernel_scale_);
        constexpr auto kPI = static_cast<Dtype>(M_PI);
        // compute the feature matrix
        // |a-b|^2 = |a|^2 + |b|^2 - 2 * a^T b
        // (1, M, Dim) -> (1, M)
        torch::Tensor diff = points_tensor - m_map_centers_.index({map_idx});  // (N, Dim)
        const torch::Tensor sq_norm_a = diff.square().sum(1, true);            // (N, 1)
        const torch::Tensor sq_norm_b = m_hinged_points_.square().sum(2);      // (1, M)
        // (N, 1) + (1, M) -> (N, M)
        torch::Tensor phi = sq_norm_a + sq_norm_b -
                            2 * torch::einsum("ij,lj->il", {diff, m_hinged_points_.squeeze(0)});
        phi = torch::exp(-gamma * phi);
        phi = torch::cat({phi, torch::ones({phi.size(0), 1}, phi.options())}, 1);

        if (faster) {  // assume sigma is very small; we can use the mean directly
            logodd_tensor = phi.matmul(mu);  // (N, 1)
        } else if (m_setting_->diagonal_sigma) {
            t1 = phi.matmul(mu);  // (N, 1)
            t2 = 1.0f + phi.square().matmul(sigma) * (kPI / 8.0f);
            t3 = t2.sqrt();
            logodd_tensor = t1 / t3;
        } else {
            t1 = phi.matmul(mu);  // (N, 1)
            t2 = 1.0f + (kPI / 8.0f) * torch::einsum("ij,ij->i", {phi, phi.matmul(sigma)});
            t2 = t2.unsqueeze(1);  // (N, 1)
            t3 = t2.sqrt();        // (N, 1)
            logodd_tensor = t1 / t3;
        }

        torch::Tensor prob_tensor;  // (N, )
        if (logodd) {
            prob_tensor = logodd_tensor;
        } else {
            prob_tensor = torch::sigmoid(logodd_tensor);
        }
        if (prob_occupied.size() < n_points) { prob_occupied.resize(n_points); }
        std::memcpy(
            prob_occupied.data(),
            prob_tensor.to(torch::kCPU).data_ptr<Dtype>(),
            n_points * sizeof(Dtype));

        if (!compute_gradient) { return; }

        diff = diff.unsqueeze(1) - m_hinged_points_;  // (N, 1, Dim) - (1, M, Dim) -> (N, M, Dim)
        diff = torch::cat(
            {
                diff,
                torch::zeros({n_points, 1, Dim}, diff.options()),
            },
            1);
        torch::Tensor grad_phi_x = -2.0f * gamma * diff * phi.unsqueeze(2);  // (N, M+1, Dim)
        torch::Tensor grads;
        if (faster) {  // (N, Dim)
            grads = torch::einsum("ijk,j->ik", {grad_phi_x, mu.squeeze(1)});
            if (gradient_with_sigmoid) {
                if (logodd) {
                    (void) grads.mul_(torch::sigmoid(prob_tensor));
                } else {
                    (void) grads.mul_(prob_tensor);
                }
            }
        } else {
            torch::Tensor grad_phi;  // (N, M+1)
            if (m_setting_->diagonal_sigma) {
                // (1, M+1) - (N, 1) * ((1, M+1) * (N, M+1)) / (N, 1) -> (N, M+1)
                grad_phi = mu.view({1, -1}) -
                           0.125f * kPI * (logodd_tensor / t2) * (sigma.view({1, -1}) * phi);
            } else {
                // (1, M+1) - ((N, 1) / (N, 1)) * ((N, M+1) @ (M+1, M+1)) -> (N, M+1)
                grad_phi =
                    mu.view({1, -1}) - 0.125f * kPI * (logodd_tensor / t2) * (phi.matmul(sigma));
            }
            grads = torch::einsum("ijk,ij->ik", {grad_phi_x, grad_phi}) / t3;
            if (gradient_with_sigmoid) {
                if (logodd) {
                    (void) grads.mul_(torch::sigmoid(prob_tensor));
                } else {
                    (void) grads.mul_(prob_tensor);
                }
            }
        }

        if (gradient.cols() < n_points) { gradient.resize(Dim, n_points); }
        std::memcpy(
            gradient.data(),
            grads.to(torch::kCPU).data_ptr<Dtype>(),
            n_points * Dim * sizeof(Dtype));
    }

    template<typename Dtype, int Dim>
    torch::Tensor
    BayesianHilbertMapTorch<Dtype, Dim>::ToBlockDiagonal(
        torch::Tensor indices,
        const torch::Tensor &values,
        const long b,
        const long m,
        const long n) const {
        indices[1] += indices[0] * m;
        indices[2] += indices[0] * n;
        indices = indices.index({torch::indexing::Slice(1, 3)});
        return torch::sparse_coo_tensor(
            indices,
            values,
            {b * m, b * n},
            torch::TensorOptions().dtype(values.dtype()).device(values.device()),
            true);
    }

    template<typename Dtype, int Dim>
    torch::Tensor
    BayesianHilbertMapTorch<Dtype, Dim>::ToBatched(
        torch::Tensor indices,
        const torch::Tensor &values,
        const long b,
        const long m,
        const long n) const {
        indices = torch::stack(
            {
                indices[0].floor_divide(m),
                indices[0].remainder(m),
                indices[1].remainder(n),
            },
            0);
        return torch::sparse_coo_tensor(indices, values, {b, m, n});
    }

    template<typename Dtype, int Dim>
    torch::Tensor
    BayesianHilbertMapTorch<Dtype, Dim>::CollapseDim(
        torch::Tensor indices,
        const torch::Tensor &values,
        const long b,
        const long m,
        const long n,
        const long dim) const {
        if (dim == 0) {
            (void) indices[0].remainder_(m);
            return torch::sparse_coo_tensor(indices, values, {m, b * n});
        }
        if (dim == 1) {
            (void) indices[1].remainder_(n);
            return torch::sparse_coo_tensor(indices, values, {b * m, n});
        }
        ERL_FATAL("CollapseDim: dim should be 0 or 1, but got {}.", dim);
    }

    template class BayesianHilbertMapTorch<float, 2>;
    template class BayesianHilbertMapTorch<float, 3>;
    template class BayesianHilbertMapTorch<double, 2>;
    template class BayesianHilbertMapTorch<double, 3>;
}  // namespace erl::geometry
#endif
