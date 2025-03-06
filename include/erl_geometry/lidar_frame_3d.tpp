#pragma once

template<typename Dtype>
YAML::Node
LidarFrame3D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
    YAML::Node node = Super::Setting::YamlConvertImpl::encode(setting);
    node["azimuth_min"] = setting.azimuth_min;
    node["azimuth_max"] = setting.azimuth_max;
    node["elevation_min"] = setting.elevation_min;
    node["elevation_max"] = setting.elevation_max;
    node["num_azimuth_lines"] = setting.num_azimuth_lines;
    node["num_elevation_lines"] = setting.num_elevation_lines;
    return node;
}

template<typename Dtype>
bool
LidarFrame3D<Dtype>::Setting::YamlConvertImpl::decode(const YAML::Node &node, Setting &setting) {
    if (!Super::Setting::YamlConvertImpl::decode(node, setting)) { return false; }
    setting.azimuth_min = node["azimuth_min"].as<Dtype>();
    setting.azimuth_max = node["azimuth_max"].as<Dtype>();
    setting.elevation_min = node["elevation_min"].as<Dtype>();
    setting.elevation_max = node["elevation_max"].as<Dtype>();
    setting.num_azimuth_lines = node["num_azimuth_lines"].as<long>();
    setting.num_elevation_lines = node["num_elevation_lines"].as<long>();
    return true;
}

template<typename Dtype>
LidarFrame3D<Dtype>::LidarFrame3D(std::shared_ptr<Setting> setting)
    : Super(setting),
      m_setting_(std::move(setting)) {
    ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");

    VectorX azimuths = VectorX::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max);
    VectorX elevations = VectorX::LinSpaced(m_setting_->num_elevation_lines, m_setting_->elevation_min, m_setting_->elevation_max);
    const long num_azimuths = azimuths.size();
    const long num_elevations = elevations.size();
    Super::m_frame_coords_.resize(num_azimuths, num_elevations);
    ERL_ASSERTM(num_azimuths > 0, "no azimuth angle.");
    ERL_ASSERTM(num_elevations > 0, "no elevation angle.");
    Super::m_dirs_frame_.resize(num_azimuths, num_elevations);

#pragma omp parallel for collapse(2) default(none) shared(num_azimuths, num_elevations, azimuths, elevations, Eigen::Dynamic)
    for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            Vector2 &frame_coord = Super::m_frame_coords_(azimuth_idx, elevation_idx);
            frame_coord << azimuths[azimuth_idx], elevations[elevation_idx];
            Super::m_dirs_frame_(azimuth_idx, elevation_idx) << common::AzimuthElevationToDirection(frame_coord[0], frame_coord[1]);
        }
    }
}

template<typename Dtype>
void
LidarFrame3D<Dtype>::UpdateRanges(
    const Eigen::Ref<const Matrix3> &rotation,
    const Eigen::Ref<const Vector3> &translation,
    MatrixX ranges,
    const bool partition_rays) {

    Super::m_rotation_ << rotation;
    Super::m_translation_ << translation;
    Super::m_ranges_ = std::move(ranges);
    m_partitions_.clear();
    m_partitioned_ = false;
    Super::m_kd_tree_->Clear();

    const long num_azimuths = GetNumAzimuthLines();
    const long num_elevations = GetNumElevationLines();
    ERL_ASSERTM(num_azimuths == Super::m_ranges_.rows(), "num of azimuths ({}) does not match rows of ranges ({}).", num_azimuths, Super::m_ranges_.rows());
    ERL_ASSERTM(
        num_elevations == Super::m_ranges_.cols(),
        "num of elevations ({}) does not match cols of ranges ({}).",
        num_elevations,
        Super::m_ranges_.cols());

    Super::m_dirs_world_.resize(num_azimuths, num_elevations);
    Super::m_end_pts_frame_.resize(num_azimuths, num_elevations);
    Super::m_end_pts_world_.resize(num_azimuths, num_elevations);

    Super::m_mask_hit_.setConstant(num_azimuths, num_elevations, false);

    Super::m_hit_ray_indices_.clear();
    Super::m_hit_ray_indices_.reserve(num_azimuths * num_elevations);
    Super::m_hit_points_world_.clear();
    Super::m_hit_points_world_.reserve(num_azimuths * num_elevations);

#pragma omp parallel for default(none) shared(num_azimuths, num_elevations, Eigen::Dynamic)
    for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            Dtype &range = Super::m_ranges_(azimuth_idx, elevation_idx);
            if (range == 0 || !std::isfinite(range)) { continue; }  // zero, nan or inf depth! Not allowed.

            // directions
            const Vector3 &dir_frame = Super::m_dirs_frame_(azimuth_idx, elevation_idx);
            Super::m_dirs_world_(azimuth_idx, elevation_idx) << Super::m_rotation_ * dir_frame;

            // end points
            Vector3 &end_pt_frame = Super::m_end_pts_frame_(azimuth_idx, elevation_idx);
            end_pt_frame << range * dir_frame;
            Super::m_end_pts_world_(azimuth_idx, elevation_idx) << Super::m_rotation_ * end_pt_frame + Super::m_translation_;

            // max valid range
            if (range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) { continue; }
            Super::m_mask_hit_(azimuth_idx, elevation_idx) = true;
        }
    }

    Super::m_max_valid_range_ = 0.0;
    for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            if (!Super::m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
            if (const Dtype &range = Super::m_ranges_(azimuth_idx, elevation_idx); range > Super::m_max_valid_range_) { Super::m_max_valid_range_ = range; }
            Super::m_hit_ray_indices_.emplace_back(azimuth_idx, elevation_idx);
            Super::m_hit_points_world_.emplace_back(Super::m_end_pts_world_(azimuth_idx, elevation_idx));
        }
    }

    if (!partition_rays) { return; }  // do not partition rays
    PartitionRays();
}

template<typename Dtype>
bool
LidarFrame3D<Dtype>::operator==(const Super &other) const {
    if (!Super::operator==(other)) { return false; }
    const auto *other_ptr = dynamic_cast<const LidarFrame3D *>(&other);
    if (other_ptr == nullptr) { return false; }
    if (m_setting_ == nullptr && other_ptr->m_setting_ != nullptr) { return false; }
    if (m_setting_ != nullptr && (other_ptr->m_setting_ == nullptr || *m_setting_ != *other_ptr->m_setting_)) { return false; }
    if (m_partitions_.size() != other_ptr->m_partitions_.size()) { return false; }
    // TODO: compare partitions
    if (m_partitioned_ != other_ptr->m_partitioned_) { return false; }
    return true;
}

template<typename Dtype>
bool
LidarFrame3D<Dtype>::Write(const std::string &filename) const {
    ERL_INFO("Writing LidarFrame3D to file: {}", filename);
    std::filesystem::create_directories(std::filesystem::path(filename).parent_path());
    std::ofstream file(filename, std::ios_base::out | std::ios_base::binary);
    if (!file.is_open()) {
        ERL_WARN("Failed to open file: {}", filename);
        return false;
    }

    const bool success = Write(file);
    file.close();
    return success;
}

template<typename Dtype>
bool
LidarFrame3D<Dtype>::Write(std::ostream &s) const {
    if (!Super::Write(s)) {
        ERL_WARN("Failed to write parent class RangeSensorFrame3D.");
        return false;
    }
    s << kFileHeader << std::endl  //
      << "# (feel free to add / change comments, but leave the first line as it is!)" << std::endl
      << "setting" << std::endl;
    // write setting
    if (!m_setting_->Write(s)) {
        ERL_WARN("Failed to write setting.");
        return false;
    }
    // write data
    s << "partitions " << m_partitions_.size() << std::endl;
    // TODO: write partitions
    s << "partitioned " << m_partitioned_ << std::endl;
    s << "end_of_LidarFrame3D" << std::endl;
    return s.good();
}

template<typename Dtype>
bool
LidarFrame3D<Dtype>::Read(const std::string &filename) {
    ERL_INFO("Reading LidarFrame3D from file: {}", std::filesystem::absolute(filename));
    std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
    if (!file.is_open()) {
        ERL_WARN("Failed to open file: {}", filename.c_str());
        return false;
    }

    const bool success = Read(file);
    file.close();
    return success;
}

template<typename Dtype>
bool
LidarFrame3D<Dtype>::Read(std::istream &s) {
    if (!Super::Read(s)) {
        ERL_WARN("Failed to read parent class RangeSensorFrame3D.");
        return false;
    }

    if (!s.good()) {
        ERL_WARN("Input stream is not ready for reading");
        return false;
    }

    // check if the first line is valid
    std::string line;
    std::getline(s, line);
    if (line.compare(0, kFileHeader.length(), kFileHeader) != 0) {  // check if the first line is valid
        ERL_WARN("Header does not start with \"{}\"", kFileHeader.c_str());
        return false;
    }

    auto skip_line = [&s]() {
        char c;
        do { c = static_cast<char>(s.get()); } while (s.good() && c != '\n');
    };

    static const char *tokens[] = {
        "setting",
        "partitions",
        "partitioned",
        "end_of_LidarFrame3D",
    };

    // read data
    std::string token;
    int token_idx = 0;
    while (s.good()) {
        s >> token;
        if (token.compare(0, 1, "#") == 0) {
            skip_line();  // comment line, skip forward until end of line
            continue;
        }
        // non-comment line
        if (token != tokens[token_idx]) {
            ERL_WARN("Expected token {}, got {}.", tokens[token_idx], token);  // check token
            return false;
        }
        // reading state machine
        switch (token_idx) {
            case 0: {  // setting
                skip_line();
                if (!m_setting_->Read(s)) {
                    ERL_WARN("Failed to read setting.");
                    return false;
                }
                break;
            }
            case 1: {  // partitions
                long num_partitions;
                s >> num_partitions;
                m_partitions_.resize(num_partitions);
                skip_line();
                // TODO: read partitions
                break;
            }
            case 2: {  // partitioned
                s >> m_partitioned_;
                break;
            }
            case 3: {  // end_of_LidarFrame3D
                skip_line();
                return true;
            }
            default: {  // should not reach here
                ERL_FATAL("Internal error, should not reach here.");
            }
        }
        ++token_idx;
    }
    ERL_WARN("Failed to read LidarFrame3D. Truncated file?");
    return false;  // should not reach here
}

template<typename Dtype>
void
LidarFrame3D<Dtype>::PartitionRays() {
    const long num_azimuths = GetNumAzimuthLines();
    const long num_elevations = GetNumElevationLines();
    // detect discontinuities, out-of-max-range measurements
    Super::m_mask_continuous_.setConstant(num_azimuths, num_elevations, true);
    Super::m_mask_continuous_.row(0).setConstant(false);
    Super::m_mask_continuous_.col(0).setConstant(false);
    Super::m_mask_continuous_.template bottomRows<1>().setConstant(false);
    Super::m_mask_continuous_.template rightCols<1>().setConstant(false);
    Dtype rolling_range_diff = 0.0;
    const Dtype gamma1 = m_setting_->rolling_diff_discount;
    const Dtype gamma2 = 1 - gamma1;
    for (long elevation_idx = 0; elevation_idx < num_elevations; ++elevation_idx) {
        for (long azimuth_idx = 0; azimuth_idx < num_azimuths; ++azimuth_idx) {
            if (!Super::m_mask_hit_(azimuth_idx, elevation_idx)) { continue; }
            const Vector2 &frame_coord = Super::m_frame_coords_(azimuth_idx, elevation_idx);
            const Dtype azimuth = frame_coord[0];
            const Dtype elevation = frame_coord[1];
            const Dtype range = Super::m_ranges_(azimuth_idx, elevation_idx);
            if (azimuth_idx == 0) {
                if (elevation_idx == 0) { continue; }
                const Dtype range_diff = std::abs(  //
                    (range - Super::m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - Super::m_frame_coords_(azimuth_idx, elevation_idx - 1)[1]));
                if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { Super::m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                if (rolling_range_diff == 0.0) { rolling_range_diff = range_diff; }
                rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
            } else {
                Dtype range_diff = std::abs(  //
                    (range - Super::m_ranges_(azimuth_idx - 1, elevation_idx)) / (azimuth - Super::m_frame_coords_(azimuth_idx - 1, elevation_idx)[0]));
                if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { Super::m_mask_continuous_(azimuth_idx - 1, elevation_idx) = false; }
                rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
                if (elevation_idx == 0) { continue; }
                range_diff = std::abs(  //
                    (range - Super::m_ranges_(azimuth_idx, elevation_idx - 1)) / (elevation - Super::m_frame_coords_(azimuth_idx, elevation_idx - 1)[1]));
                if (range_diff > m_setting_->discontinuity_factor * rolling_range_diff) { Super::m_mask_continuous_(azimuth_idx, elevation_idx - 1) = false; }
                rolling_range_diff = gamma1 * rolling_range_diff + gamma2 * range_diff;
            }
        }
    }

    throw std::runtime_error("Implementation for partitioning rays is not complete.");
}
