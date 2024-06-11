from erl_common.storage import GridMapInfo2D
from erl_geometry import OccupancyQuadtree
from erl_geometry import CityStreetMap
import cv2
import os
import numpy as np
from collections import namedtuple

UserData = namedtuple("UserData", ["mouse_fixed", "image", "quadtree", "drawer"])


def mouse_callback(event, x, y, flags, userdata: UserData):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Left button down at ({}, {})".format(x, y))
        userdata.mouse_fixed = not userdata.mouse_fixed
        return

    if event == cv2.EVENT_MOUSEMOVE:
        if userdata.mouse_fixed:
            return

        # we should use the map_info of the drawer instead of the one to build the quadtree
        # because padding may be added to the quadtree
        map_info = userdata.drawer.grid_map_info

        position = np.array(
            [
                map_info.grid_to_meter_for_value(x, 0),
                map_info.grid_to_meter_for_value(map_info.shape_at(1) - y, 1),
            ]
        )
        rotation = np.eye(2)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        results: dict = userdata.quadtree.cast_rays(
            position,
            rotation,
            angles,
            ignore_unknown=False,  # if True, unknown cells are treated as free cells
            max_range=-1.0,  # -1.0 means infinite range
            prune_rays=False,  # if True, the rays are pruned by the leaves of the quadtree
            parallel=True,  # enable parallel computation
        )
        # there are 3 keys: hit_ray_indices, hit_positions and hit_nodes
        hit_positions: list = results["hit_positions"]
        image: np.ndarray = userdata.image.copy()  # copy the image to avoid modifying the original image
        for hit_position in hit_positions:
            cv2.line(
                image,
                np.array([x, y]),
                np.array(
                    [
                        map_info.meter_to_grid_for_value(hit_position[0], 0),
                        map_info.shape_at(1) - map_info.meter_to_grid_for_value(hit_position[1], 1),
                    ]
                ),
                [0, 0, 255, 255],  # BGR, red
                1,  # thickness
            )
        cv2.imshow("OccupancyQuadtree", image)


def main():
    test_dir = os.path.dirname(os.path.abspath(__file__))
    city_street_map_file = os.path.join(test_dir, "..", "gtest", "Berlin_0_1024.map")
    city_street_map = CityStreetMap.load(city_street_map_file)
    map_info = GridMapInfo2D(
        map_shape=np.array([1024, 1024]),
        min=np.array([-10.0, -10.0]),  # min and max are in meters, current resolution is 0.1m
        max=np.array([92.4, 92.4]),
    )
    # padding=1: pad the map with 1-pixel-wide obstacles
    quadtree = OccupancyQuadtree(map_info, city_street_map, occupied_threshold=0.0, padding=1)
    drawer_setting = OccupancyQuadtree.Drawer.Setting()
    drawer_setting.resolution = 0.075  # 0.075m per pixel, smaller than the quadtree resolution
    drawer_setting.area_min = np.array([-10.5, -10.5])
    drawer_setting.area_max = np.array([93.0, 93.0])
    drawer_setting.border_color = np.array([255, 0, 0])  # BGR
    drawer = OccupancyQuadtree.Drawer(drawer_setting, quadtree)
    image = drawer.draw_tree()

    cv2.imshow("OccupancyQuadtree", image)
    userdata = UserData(False, image, quadtree, drawer)
    cv2.setMouseCallback("OccupancyQuadtree", mouse_callback, userdata)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
