#pragma once

template<typename Dtype>
YAML::Node
CameraIntrinsic<Dtype>::YamlConvertImpl::encode(const CameraIntrinsic &intrinsic) {
    YAML::Node node;
    node["image_height"] = intrinsic.image_height;
    node["image_width"] = intrinsic.image_width;
    node["camera_fx"] = intrinsic.camera_fx;
    node["camera_fy"] = intrinsic.camera_fy;
    node["camera_cx"] = intrinsic.camera_cx;
    node["camera_cy"] = intrinsic.camera_cy;
    return node;
}

template<typename Dtype>
bool
CameraIntrinsic<Dtype>::YamlConvertImpl::decode(const YAML::Node &node, CameraIntrinsic &intrinsic) {
    if (!node.IsMap()) { return false; }
    intrinsic.image_height = node["image_height"].as<long>();
    intrinsic.image_width = node["image_width"].as<long>();
    intrinsic.camera_fx = node["camera_fx"].as<Dtype>();
    intrinsic.camera_fy = node["camera_fy"].as<Dtype>();
    intrinsic.camera_cx = node["camera_cx"].as<Dtype>();
    intrinsic.camera_cy = node["camera_cy"].as<Dtype>();
    return true;
}

template<typename Dtype>
void
CameraIntrinsic<Dtype>::ComputeFrameDirections(Eigen::MatrixX<Vector3> &dirs) const {
    dirs.resize(image_height, image_width);

    // convert from image coordinates to normalized image coordinates
    // [u, v, 1] = K * [x, y, 1], where K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    // x = (u - cx) / fx, y = (v - cy) / fy

    const Dtype fx_inv = 1.0 / camera_fx;
    const Dtype fy_inv = 1.0 / camera_fy;

#pragma omp parallel for default(none) shared(fx_inv, fy_inv, dirs, Eigen::Dynamic)
    for (long u = 0; u < image_width; ++u) {
        const Dtype xu = (static_cast<Dtype>(u) - camera_cx) * fx_inv;
        for (long v = 0; v < image_height; ++v) {
            const Dtype yv = (static_cast<Dtype>(v) - camera_cy) * fy_inv;
            // normalized image coordinates
            // v <--> image coordinate y, u <--> image coordinate x
            Vector3 &dir_frame = dirs(v, u);
            dir_frame << xu, yv, 1.0;
            dir_frame.normalize();
        }
    }
}

template<typename Dtype>
void
CameraIntrinsic<Dtype>::ComputeFrameDirections(Eigen::MatrixX<Vector2> &coords, Eigen::MatrixX<Vector3> &dirs) const {
    coords.resize(image_height, image_width);
    dirs.resize(image_height, image_width);

    // convert from image coordinates to normalized image coordinates
    // [u, v, 1] = K * [x, y, 1], where K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    // x = (u - cx) / fx, y = (v - cy) / fy

    const Dtype fx_inv = 1.0 / camera_fx;
    const Dtype fy_inv = 1.0 / camera_fy;

#pragma omp parallel for default(none) shared(fx_inv, fy_inv, coords, dirs, Eigen::Dynamic)
    for (long u = 0; u < image_width; ++u) {
        const Dtype xu = (static_cast<Dtype>(u) - camera_cx) * fx_inv;
        for (long v = 0; v < image_height; ++v) {
            const Dtype yv = (static_cast<Dtype>(v) - camera_cy) * fy_inv;
            // normalized image coordinates
            // v <--> image coordinate y, u <--> image coordinate x
            coords(v, u) << yv, xu;  // frame coords
            Vector3 &dir_frame = dirs(v, u);
            dir_frame << xu, yv, 1.0;
            dir_frame.normalize();
        }
    }
}

template<typename Dtype>
void
CameraIntrinsic<Dtype>::ConvertDepthToDistance(const Matrix &depth, Matrix &distance) const {
    ERL_ASSERTM(depth.rows() == image_height, "depth image height ({}) does not match setting ({}).", depth.rows(), image_height);
    ERL_ASSERTM(depth.cols() == image_width, "depth image width ({}) does not match setting ({}).", depth.cols(), image_width);

    distance.resize(image_height, image_width);

    const Dtype fx_inv = 1.0 / camera_fx;
    const Dtype fy_inv = 1.0 / camera_fy;

#pragma omp parallel for default(none) shared(fx_inv, fy_inv, depth, distance, Eigen::Dynamic)
    for (long u = 0; u < image_width; ++u) {
        const Dtype xu = (static_cast<Dtype>(u) - camera_cx) * fx_inv;
        Dtype *distance_ptr = distance.col(u).data();
        const Dtype *depth_ptr = depth.col(u).data();
        for (long v = 0; v < image_height; ++v) {
            const Dtype yv = (static_cast<Dtype>(v) - camera_cy) * fy_inv;
            distance_ptr[v] = depth_ptr[v] * std::sqrt(xu * xu + yv * yv + 1.0);
        }
    }
}

template<typename Dtype>
void
CameraIntrinsic<Dtype>::ConvertDistanceToDepth(const Matrix &distance, Matrix &depth) const {
    ERL_ASSERTM(distance.rows() == image_height, "distance image height ({}) does not match setting ({}).", distance.rows(), image_height);
    ERL_ASSERTM(distance.cols() == image_width, "distance image width ({}) does not match setting ({}).", distance.cols(), image_width);

    depth.resize(image_height, image_width);

    const Dtype fx_inv = 1.0 / camera_fx;
    const Dtype fy_inv = 1.0 / camera_fy;

#pragma omp parallel for default(none) shared(fx_inv, fy_inv, depth, distance, Eigen::Dynamic)
    for (long u = 0; u < image_width; ++u) {
        const Dtype xu = (static_cast<Dtype>(u) - camera_cx) * fx_inv;
        Dtype *depth_ptr = depth.col(u).data();
        const Dtype *distance_ptr = distance.col(u).data();
        for (long v = 0; v < image_height; ++v) {
            const Dtype yv = (static_cast<Dtype>(v) - camera_cy) * fy_inv;
            depth_ptr[v] = distance_ptr[v] / std::sqrt(xu * xu + yv * yv + 1.0);
        }
    }
}

template<typename Dtype>
void
CameraIntrinsic<Dtype>::ConvertRgbdToPointCloud(
    const Matrix &depth,
    const cv::Mat &rgb,
    const std::optional<Matrix4> &optical_pose,
    std::vector<Vector3> &points,
    std::vector<Vector3> &colors) const {
    ERL_ASSERTM(depth.rows() == image_height, "depth image height ({}) does not match setting ({}).", depth.rows(), image_height);
    ERL_ASSERTM(depth.cols() == image_width, "depth image width ({}) does not match setting ({}).", depth.cols(), image_width);

    if (!rgb.empty()) {
        ERL_ASSERTM(rgb.rows == image_height, "rgb image height ({}) does not match setting ({}).", rgb.rows, image_height);
        ERL_ASSERTM(rgb.cols == image_width, "rgb image width ({}) does not match setting ({}).", rgb.cols, image_width);
        ERL_ASSERTM(rgb.depth() == CV_8U, "rgb image depth ({}) should be CV_8U.", rgb.depth());
        ERL_ASSERTM(rgb.channels() == 3, "rgb image channels ({}) should be 3.", rgb.channels());
        colors.resize(depth.size());
    }

    points.resize(depth.size());

    const Dtype fx_inv = 1.0 / camera_fx;
    const Dtype fy_inv = 1.0 / camera_fy;
    const bool has_optical_pose = optical_pose.has_value();
    Eigen::Matrix3d rotation;
    Vector3 translation;
    if (has_optical_pose) {
        rotation = optical_pose->template topLeftCorner<3, 3>();
        translation = optical_pose->template topRightCorner<3, 1>();
    }

#pragma omp parallel for default(none) shared(fx_inv, fy_inv, depth, rgb, has_optical_pose, rotation, translation, points, colors, Eigen::Dynamic)
    for (long u = 0; u < image_width; ++u) {
        const Dtype xu = (static_cast<Dtype>(u) - camera_cx) * fx_inv;
        const Dtype *depth_ptr = depth.col(u).data();
        for (long v = 0, i = u * image_height; v < image_height; ++v, ++i) {
            const Dtype &d = depth_ptr[v];
            Vector3 &point = points[i];
            if (!std::isfinite(d) || d <= 0) { continue; }  // non-positive or nan depth! Not allowed.
            const Dtype yv = (static_cast<Dtype>(v) - camera_cy) * fy_inv;
            point[0] = depth_ptr[v] * xu;
            point[1] = depth_ptr[v] * yv;
            point[2] = depth_ptr[v];
            if (has_optical_pose) { point = rotation * point + translation; }
            if (!rgb.empty()) {
                const auto &color = rgb.at<cv::Vec3b>(v, u);
                Vector3 &color_point = colors[i];
                color_point[0] = static_cast<Dtype>(color[0]) / 255.0;
                color_point[1] = static_cast<Dtype>(color[1]) / 255.0;
                color_point[2] = static_cast<Dtype>(color[2]) / 255.0;
            }
        }
    }

    // remove invalid points
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Dtype &d = depth.data()[i];
        if (std::isfinite(d) && d > 0) { continue; }
        std::size_t j = points.size() - 1;
        while (j > i) {
            if (const Dtype &d2 = depth.data()[j]; std::isfinite(d2) && d2 > 0) { break; }
            --j;
        }
        points[i] = points[j];
        points.pop_back();
        if (!rgb.empty()) {
            colors[i] = colors[j];
            colors.pop_back();
        }
    }
}
