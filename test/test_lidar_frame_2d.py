import numpy as np
import vedo
from erl_common.storage import GridMapInfo2D
from erl_common.vedo_ext.lines import lidar_rays
from erl_common.vedo_ext.picture import image_mesh
from erl_geometry.house_expo import HouseExpoMap
from erl_geometry.house_expo.list_data import get_map_and_traj_files
from erl_geometry import Lidar2D
from erl_geometry import LidarFrame2D

# env
map_file, traj_file = get_map_and_traj_files()[1451]
env = HouseExpoMap(map_file)
lidar = Lidar2D(env.meter_space)

# visualization
vertices = env.meter_space.surface.vertices
grid_map_info = GridMapInfo2D(
    min=np.min(vertices, axis=1),
    max=np.max(vertices, axis=1),
    resolution=np.array([0.01, 0.01]),
    padding=np.array([10, 10]),
)
map_image = env.meter_space.generate_map_image(grid_map_info)

# vedo
plt = vedo.Plotter()
vedo_objs = []


def mouse_move_callback(event):
    if event is None:
        return
    if not event.actor:
        return
    if event.at != 0:
        return

    p = event.picked3d
    if p is None:
        return

    if len(vedo_objs) > 0:
        plt.remove(*vedo_objs)
        vedo_objs.clear()

    pos = np.array(p[:2])
    lidar.translation = pos
    lidar.set_rotation(np.pi / 3)

    angles = lidar.angles
    ranges = lidar.scan(parallel=True)

    setting = LidarFrame2D.Setting()
    setting.discontinuity_factor = 10  # 4  # 8  # 15  # 29  # 57.1  # 571.4
    setting.rolling_diff_discount = 0.6
    frame = LidarFrame2D(setting)
    frame.update(lidar.rotation, lidar.translation, angles, ranges, partition_rays=True)
    colors = [
        "red",
        "green",
        "blue",
        "yellow",
        "cyan",
        "magenta",
        "black",
        "orange",
        "purple",
        "brown",
        "pink",
        "gray",
    ]
    for i, partition in enumerate(frame.partitions):
        dirs_world = frame.ray_directions_in_world[:, partition.index_begin : partition.index_end]
        ranges = frame.ranges[partition.index_begin : partition.index_end]
        vedo_objs.append(lidar_rays(frame.translation_vector, dirs_world, ranges, colors[i]))
    plt.add(*vedo_objs)
    plt.render()


def main():
    plt.add_callback("mouse move", mouse_move_callback)
    plt.show(
        image_mesh(
            vedo.Picture(map_image, flip=True), grid_map_info.get_dim_lin_space(0), grid_map_info.get_dim_lin_space(1)
        ),
        axes=1,
    )


if __name__ == "__main__":
    main()
