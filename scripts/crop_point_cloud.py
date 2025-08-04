import open3d as o3d
import numpy as np


def main():
    pcd = o3d.io.read_point_cloud("/home/daizhirui/Data/NewerColledge/all_points.ply")
    bounding_box: o3d.geometry.OrientedBoundingBox = pcd.get_oriented_bounding_box()
    margin = 0.1
    bounding_box.extent = np.asarray(bounding_box.extent) + margin

    pcd = o3d.io.read_point_cloud(
        "/home/daizhirui/DataArchive/NewerColledge/2021-ouster-os0-128-alphasense/prior map/new-college-combined-1cm-v2.ply"
    )
    pcd = pcd.crop(bounding_box)
    o3d.io.write_point_cloud("/home/daizhirui/Data/NewerColledge/new-college-combined-1cm-v2-quad-cropped.ply", pcd)


if __name__ == "__main__":
    main()
