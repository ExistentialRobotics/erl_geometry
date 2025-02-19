#pragma once

template<typename Dtype>
typename RangeSensor3D<Dtype>::Matrix
RangeSensor3D<Dtype>::Scan(
    const Eigen::Ref<const Matrix3> &orientation,
    const Eigen::Ref<const Vector3> &translation,
    const bool add_noise,
    const Dtype noise_stddev,
    bool cache_normals) {

    Eigen::MatrixX<Vector3> directions = GetRayDirectionsInFrame();
    const long n_rows = directions.rows();
    const long n_cols = directions.cols();
    Matrix scales(n_rows, n_cols);
    const Eigen::Vector3f &ray_start = translation.template cast<float>();  // float32, will be copied to tensor
    // column major
    open3d::core::Tensor rays({n_rows, n_cols, 6}, open3d::core::Dtype::Float32);
    auto *rays_ptr = rays.GetDataPtr<float>();
#pragma omp parallel for default(none) shared(n_rows, n_cols, orientation, directions, scales, ray_start, rays_ptr)
    for (long v = 0; v < n_cols; ++v) {
        const long base0 = v * n_rows;
        for (long u = 0; u < n_rows; ++u) {
            long base1 = base0 + u;

            Eigen::Vector3f direction = (orientation * directions.data()[base1]).template cast<float>();
            Dtype &scale = scales.data()[base1];
            scale = static_cast<Dtype>(direction.norm());  // keep scale for later use

            base1 *= 6;
            rays_ptr[base1 + 0] = ray_start[0];
            rays_ptr[base1 + 1] = ray_start[1];
            rays_ptr[base1 + 2] = ray_start[2];

            // may not be normalized, e.g. depth camera uses camera normalized coordinates.
            rays_ptr[base1 + 3] = direction[0] / scale;
            rays_ptr[base1 + 4] = direction[1] / scale;
            rays_ptr[base1 + 5] = direction[2] / scale;
        }
    }

    // cast rays
    const std::unordered_map<std::string, open3d::core::Tensor> cast_results = m_scene_->CastRays(rays);
    const std::vector<float> ranges = cast_results.at("t_hit").ToFlatVector<float>();
    const std::vector<uint32_t> geometry_ids = cast_results.at("geometry_ids").ToFlatVector<uint32_t>();
    const std::vector<float> normals = cache_normals ? cast_results.at("primitive_normals").ToFlatVector<float>() : std::vector<float>{};
    const uint32_t invalid_id = open3d::t::geometry::RaycastingScene::INVALID_ID();

    // copy results to Eigen matrix
    if (cache_normals && (m_normals_.rows() != n_rows || m_normals_.cols() != n_cols)) { m_normals_ = Eigen::MatrixX<Vector3>(n_rows, n_cols); }
    Matrix ranges_mat(n_rows, n_cols);
    std::vector<uint64_t> random_seeds(n_cols);
    for (long i = 0; i < n_cols; ++i) { random_seeds[i] = erl::common::g_random_engine(); }
#pragma omp parallel for default(none) \
    shared(n_rows, n_cols, ranges, ranges_mat, geometry_ids, normals, scales, invalid_id, random_seeds, add_noise, noise_stddev, cache_normals)
    for (long v = 0; v < n_cols; ++v) {
        const long base0 = v * n_rows;
        std::mt19937_64 generator(random_seeds[v]);
        std::normal_distribution<Dtype> distribution(0, noise_stddev);
        for (long u = 0; u < n_rows; ++u) {
            if (const long i = base0 + u; geometry_ids[i] == invalid_id) {
                ranges_mat(u, v) = std::numeric_limits<Dtype>::infinity();
                if (cache_normals) { m_normals_(u, v).setZero(); }
            } else {
                Dtype &range = ranges_mat(u, v);
                range = ranges[i] / scales.data()[i];
                if (add_noise) { range += distribution(generator); }
                if (cache_normals) {
                    Vector3 &normal = m_normals_(u, v);
                    const long ii = i * 3;
                    normal[0] = static_cast<Dtype>(normals[ii + 0]);
                    normal[1] = static_cast<Dtype>(normals[ii + 1]);
                    normal[2] = static_cast<Dtype>(normals[ii + 2]);
                    normal.normalize();
                }
            }
        }
    }

    return ranges_mat;
}
