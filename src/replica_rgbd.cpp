#include "erl_geometry/replica_rgbd.hpp"

#include "erl_geometry/rgbd_camera_3d.hpp"

#include <open3d/io/TriangleMeshIO.h>

namespace erl::geometry {
    ReplicaRgbd::ReplicaRgbd(
        std::filesystem::path directory,
        std::string scene_name,
        bool load_as_bgr,
        bool generate_rgbd)
        : m_directory_(std::move(directory)),
          m_scene_name_(std::move(scene_name)),
          m_load_as_bgr_(load_as_bgr) {
        ERL_ASSERTM(
            std::filesystem::exists(m_directory_),
            "Directory does not exist: {}",
            m_directory_.string());

        m_mesh_path_ = m_directory_ / (m_scene_name_ + "_mesh.ply");
        ERL_ASSERTM(
            std::filesystem::exists(m_mesh_path_),
            "Mesh file does not exist: {}",
            m_mesh_path_.string());

        m_traj_path_ = m_directory_ / m_scene_name_ / "traj.txt";
        ERL_ASSERTM(
            std::filesystem::exists(m_traj_path_),
            "Trajectory file does not exist: {}",
            m_traj_path_.string());

        m_rgbd_dir_ = m_directory_ / m_scene_name_ / "results";
        ERL_ASSERTM(
            generate_rgbd || std::filesystem::exists(m_rgbd_dir_),
            "RGBD directory does not exist: {}",
            m_rgbd_dir_.string());

        auto mesh = open3d::io::CreateMeshFromFile(m_mesh_path_.string());
        ERL_ASSERTM(
            mesh != nullptr && !mesh->vertices_.empty(),
            "Failed to load mesh from file: {}",
            m_mesh_path_.string());
        m_map_min_ = mesh->GetMinBound();
        m_map_max_ = mesh->GetMaxBound();

        m_pose_data_ = common::LoadEigenMatrixFromTextFile<double>(
            m_traj_path_,
            common::EigenTextFormat::kDefaultFmt,
            true);

        if (!generate_rgbd) { return; }

        // generate RGBD images
        const long num_frames = m_pose_data_.cols();
        auto camera_setting = std::make_shared<RgbdCamera3Dd::Setting>();
        camera_setting->image_width = kImageWidth;
        camera_setting->image_height = kImageHeight;
        camera_setting->camera_cx = kCameraCx;
        camera_setting->camera_cy = kCameraCy;
        camera_setting->camera_fx = kCameraFx;
        camera_setting->camera_fy = kCameraFy;
        RgbdCamera3Dd camera(camera_setting);
        camera.AddMesh(m_mesh_path_.string());
        using Pose = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
        for (long i = 0; i < num_frames; ++i) {
            Eigen::Map<Pose> pose(m_pose_data_.col(i).data(), 4, 4);
            const Eigen::Matrix3d orientation = pose.topLeftCorner<3, 3>();
            const Eigen::Vector3d translation = pose.topRightCorner<3, 1>();
            auto [rgb, depth] = camera.Scan(orientation, translation);
            rgb.convertTo(rgb, CV_8UC3, 255.0);  // F32C3 -> U8C3
            cv::imwrite(m_rgbd_dir_ / fmt::format("frame{:06d}.jpg", i), rgb);
            depth *= kDepthScale;
            depth.convertTo(depth, CV_16UC1);  // F32C1 -> U16C1
            cv::imwrite(m_rgbd_dir_ / fmt::format("depth{:06d}.png", i), depth);
        }
    }

    ReplicaRgbd::Frame
    ReplicaRgbd::operator[](long index) const {
        Frame frame;
        frame.sequence_number = index;

        using Pose = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
        Eigen::Map<const Pose> pose(m_pose_data_.col(index).data(), 4, 4);
        frame.rotation = pose.topLeftCorner<3, 3>();
        frame.translation = pose.topRightCorner<3, 1>();

        const auto depth_file = m_rgbd_dir_ / fmt::format("depth{:06d}.png", frame.sequence_number);
        ERL_ASSERTM(
            std::filesystem::exists(depth_file),
            "Depth file does not exist: {}",
            depth_file.string());
        cv::Mat depth = cv::imread(depth_file.string(), cv::IMREAD_UNCHANGED);
        ERL_ASSERTM(!depth.empty(), "Failed to load depth image: {}", depth_file.string());
        depth.convertTo(depth, CV_64FC1);  // U16C1 -> F64C1
        cv::cv2eigen(depth, frame.depth);
        frame.depth.array() /= kDepthScale;

        cv::normalize(depth, depth, 0.0, 255.0, cv::NORM_MINMAX);
        depth.convertTo(depth, CV_8UC1);
        cv::applyColorMap(depth, frame.depth_jet, cv::COLORMAP_JET);

        const auto color_file = m_rgbd_dir_ / fmt::format("frame{:06d}.jpg", frame.sequence_number);
        ERL_ASSERTM(
            std::filesystem::exists(color_file),
            "Color file does not exist: {}",
            color_file.string());
        frame.color = cv::imread(color_file.string(), cv::IMREAD_COLOR);
        if (!m_load_as_bgr_) { cv::cvtColor(frame.color, frame.color, cv::COLOR_BGR2RGB); }
        ERL_ASSERTM(!frame.color.empty(), "Failed to load color image: {}", color_file.string());
        return frame;
    }

}  // namespace erl::geometry
