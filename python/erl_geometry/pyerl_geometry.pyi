import collections
from enum import IntEnum
from typing import Callable
from typing import Optional
from typing import Tuple
from typing import TypedDict
from typing import overload

import numpy as np
from erl_common.storage import GridMapInfo2D
from erl_common.yaml import YamlableBase

__all__ = [
    "manually_set_seed",
    "marching_square",
    "bresenham_2d",
    "compute_pixel_of_polygon_contour",
    "winding_number",
    "compute_nearest_distance_from_point_to_line_segment_2d",
    "compute_intersection_between_ray_and_segment_2d",
    "compute_intersection_between_ray_and_aabb_2d",
    "compute_intersection_between_ray_and_aabb_3d",
    "convert_path_2d_to_3d",
    "Aabb2D",
    "Aabb3D",
    "CityStreetMap",
    "NdTreeSetting",
    "OccupancyNdTreeSetting",
    "OccupancyQuadtreeBaseSetting",
    "QuadtreeKey",
    "QuadtreeKeyRay",
    "OccupancyQuadtreeNode",
    "OccupancyQuadtree",
    "OccupancyOctreeBaseSetting",
    "OctreeKey",
    "OctreeKeyRay",
    "OccupancyOctreeNode",
    "OccupancyOctree",
    "Surface2D",
    "Space2D",
    "Lidar2D",
    "LidarFramePartition2D",
    "LidarFrame2D",
    "LogOddMap2D",
    "Lidar3D",
    "DepthCamera3D",
    "LidarFramePartition3D",
    "LidarFrame3D",
    "RgbdFrame3D",
]

def manually_set_seed(seed: int) -> None: ...
def marching_square(
    img: np.ndarray[np.float64], iso_value: float
) -> Tuple[np.ndarray[np.float64], np.ndarray[np.float64], np.ndarray[np.float64]]: ...
def bresenham_2d(
    start: np.ndarray[np.float64], end: np.ndarray[np.float64], stop: Callable[[int, int], bool] = None
) -> np.ndarray[np.float64]: ...
def compute_pixel_of_polygon_contour(polygon_vertices: np.ndarray[np.int32]) -> np.ndarray[np.int32]: ...
def winding_number(p: np.ndarray[np.float64], vertices: np.ndarray[np.float64]) -> int: ...
def compute_nearest_distance_from_point_to_line_segment_2d(
    point_x: float,
    point_y: float,
    line_segment_x1: float,
    line_segment_y1: float,
    line_segment_x2: float,
    line_segment_y2: float,
) -> float: ...
def compute_intersection_between_ray_and_segment_2d(
    ray_start_point: np.ndarray[np.float64],
    ray_direction: np.ndarray[np.float64],
    segment_point1: np.ndarray[np.float64],
    segment_point2: np.ndarray[np.float64],
) -> Tuple[float, float]:
    """
    Returns:
        Tuple[float, float]: (t, distance) where intersection = t * segment_point1 + (1 - t) * segment_point2
    """
    ...

def compute_intersection_between_ray_and_aabb_2d(
    ray_start_point: np.ndarray[np.float64],
    ray_direction: np.ndarray[np.float64],
    aabb_min: np.ndarray[np.float64],
    aabb_max: np.ndarray[np.float64],
) -> Tuple[float, float, bool]: ...
def compute_intersection_between_ray_and_aabb_3d(
    ray_start_point: np.ndarray[np.float64],
    ray_direction: np.ndarray[np.float64],
    aabb_min: np.ndarray[np.float64],
    aabb_max: np.ndarray[np.float64],
) -> Tuple[float, float, bool]: ...
def convert_path_2d_to_3d(path_2d: np.ndarray, z: float) -> list[np.ndarray]: ...

class Aabb2D:
    class CornerType(IntEnum):
        kBottomLeft = 0
        kBottomRight = 1
        kTopLeft = 2
        kTopRight = 3

    def __init__(self: Aabb2D, center: np.ndarray, half_size: float): ...
    @property
    def center(self: Aabb2D) -> np.ndarray: ...
    @property
    def half_sizes(self: Aabb2D) -> float: ...
    @property
    def min(self: Aabb2D) -> np.ndarray: ...
    @property
    def max(self: Aabb2D) -> np.ndarray: ...
    @overload
    def __contains__(self, point: np.ndarray) -> bool: ...
    @overload
    def __contains__(self: Aabb2D, aabb: Aabb2D) -> bool: ...
    def corner(self: Aabb2D, corner_type: int) -> np.ndarray: ...
    def intersects(self: Aabb2D, aabb: Aabb2D) -> bool: ...

class Aabb3D:
    class CornerType(IntEnum):
        kBottomLeftFloor = 0
        kBottomRightFloor = 1
        kTopLeftFloor = 2
        kTopRightFloor = 3
        kBottomLeftCeil = 4
        kBottomRightCeil = 5
        kTopLeftCeil = 6
        kTopRightCeil = 7

    def __init__(self: Aabb2D, center: np.ndarray, half_size: float): ...
    @property
    def center(self: Aabb2D) -> np.ndarray: ...
    @property
    def half_sizes(self: Aabb2D) -> float: ...
    @property
    def min(self: Aabb2D) -> np.ndarray: ...
    @property
    def max(self: Aabb2D) -> np.ndarray: ...
    @overload
    def __contains__(self, point: np.ndarray) -> bool: ...
    @overload
    def __contains__(self: Aabb2D, aabb: Aabb2D) -> bool: ...
    def corner(self: Aabb2D, corner_type: int) -> np.ndarray: ...
    def intersects(self: Aabb2D, aabb: Aabb2D) -> bool: ...

class CityStreetMap:
    @property
    def kFree(self) -> int: ...
    @property
    def kObstacle(self) -> int: ...
    @property
    def kPassableDot(self) -> str: ...
    @property
    def kPassableG(self) -> str: ...
    @property
    def kOutOfBoundAt(self) -> str: ...
    @property
    def kOutOfBoundO(self) -> str: ...
    @property
    def kTree(self) -> str: ...
    @property
    def kSwamp(self) -> str: ...
    @property
    def kWater(self) -> str: ...
    @staticmethod
    def load_map(filename: str) -> np.ndarray: ...

    class Scene:
        bucket: int
        map: str
        map_width: int
        map_height: int
        start_x: int
        start_y: int
        goal_x: int
        goal_y: int
        optimal_length: float

    @staticmethod
    def load_scenes(filename: str) -> list[Scene]: ...

class NdTreeSetting(YamlableBase):
    resolution: float
    tree_depth: int

    def __init__(self: NdTreeSetting) -> None: ...

class OccupancyNdTreeSetting(NdTreeSetting):
    log_odd_min: float
    log_odd_max: float
    log_odd_hit: float
    log_odd_miss: float
    log_odd_occ_threshold: float
    probability_hit: float
    probability_miss: float
    probability_occupied_threshold: float

    def __init__(self: OccupancyNdTreeSetting) -> None: ...

class OccupancyQuadtreeBaseSetting(OccupancyNdTreeSetting):
    use_change_detection: bool
    use_aabb_limit: bool
    aabb: Aabb2D

    def __init__(self: OccupancyQuadtreeBaseSetting) -> None: ...

class QuadtreeKey:
    def __eq__(self, other) -> bool: ...
    def __ne__(self, other) -> bool: ...
    def __getitem__(self, item) -> int: ...

class QuadtreeKeyRay:
    def __len__(self) -> int: ...
    def __getitem__(self, item) -> QuadtreeKey: ...

class OccupancyQuadtreeNode:
    @property
    def node_type(self) -> str: ...
    @property
    def depth(self) -> int: ...
    @property
    def child_index(self) -> int: ...
    @property
    def num_children(self) -> int: ...
    @property
    def has_any_child(self) -> bool: ...
    def has_child(self, child_index: int) -> bool: ...
    def get_child(self, child_index: int) -> OccupancyQuadtreeNode: ...
    @property
    def occupancy(self) -> float: ...
    @property
    def log_odds(self) -> float: ...
    @property
    def mean_child_log_odds(self) -> float: ...
    @property
    def max_child_log_odds(self) -> float: ...
    def allow_update_log_odds(self, delta: float) -> bool: ...
    def add_log_odds(self, log_odds: float) -> None: ...

class OccupancyQuadtree:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, setting: OccupancyQuadtreeBaseSetting) -> None: ...
    @overload
    def __init__(
        self,
        map_info: GridMapInfo2D,
        image_map: np.ndarray,
        occupied_threshold: float,
        padding: int = 0,
    ) -> None: ...
    @overload
    def __init__(self, filename: str, is_binary: bool) -> None: ...
    def apply_setting(self) -> None: ...
    def write_raw(self, filename: str) -> bool: ...
    def read_raw(self, filename: str) -> bool: ...
    def write_binary(self, filename: str, prune_at_first: bool) -> bool: ...
    def read_binary(self, filename: str) -> bool: ...
    def is_node_occupied(self, node: OccupancyQuadtreeNode) -> bool: ...
    @property
    def tree_type(self) -> str: ...
    @property
    def setting(self) -> OccupancyQuadtreeBaseSetting: ...
    @property
    def is_node_collapsible(self) -> bool: ...
    def insert_point_cloud(
        self,
        points: np.ndarray[np.float64],
        sensor_origin: np.ndarray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
        discretize: bool,
    ) -> None: ...
    def insert_point_cloud_rays(
        self,
        points: np.ndarray[np.float64],
        sensor_origin: np.ndarray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
    ) -> None: ...
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, max_range: float, lazy_eval: bool) -> None: ...
    def sample_positions(self, num_positions: int) -> list[np.ndarray]: ...
    @overload
    def cast_rays(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
        angles: np.ndarray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": np.ndarray,
            "hit_positions": np.ndarray,
            "hit_nodes": list[OccupancyQuadtreeNode],
            "node_depths": list[int],
        },
    ): ...
    @overload
    def cast_rays(
        self,
        positions: np.ndarray,
        directions: np.ndarray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": np.ndarray,
            "hit_positions": np.ndarray,
            "hit_nodes": list[OccupancyQuadtreeNode],
            "node_depths": list[int],
        },
    ): ...
    def cast_ray(
        self,
        px: float,
        py: float,
        vx: float,
        vy: float,
        ignore_unknown: bool,
        max_range: float,
    ) -> TypedDict("returns", {"hit_node": OccupancyQuadtreeNode, "ex": float, "ey": float}): ...
    @overload
    def update_node(self, x: float, y: float, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    @overload
    def update_node(self, node_key: QuadtreeKey, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    @overload
    def update_node(self, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    @overload
    def update_node(self, node_key: QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    def update_inner_occupancy(self) -> None: ...
    def to_max_likelihood(self) -> None: ...
    @property
    def number_of_nodes(self) -> int: ...

    resolution: float

    @property
    def tree_depth(self) -> int: ...
    @property
    def tree_key_offset(self) -> int: ...
    @property
    def metric_min(self) -> Tuple[float, float]: ...
    @property
    def metric_max(self) -> Tuple[float, float]: ...
    @property
    def metric_min_max(self) -> Tuple[Tuple[float, float], Tuple[float, float]]: ...
    @property
    def metric_size(self) -> Tuple[float, float]: ...
    def get_node_size(self, depth: int) -> float: ...
    @property
    def number_of_leaf_nodes(self) -> int: ...
    @property
    def memory_usage(self) -> int: ...
    @property
    def memory_usage_per_node(self) -> int: ...
    @overload
    def coord_to_key(self, coordinate: float) -> int: ...
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int: ...
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey: ...
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey: ...
    @overload
    def coord_to_key_checked(self, coordinate: float) -> Optional[int]: ...
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> Optional[int]: ...
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> Optional[QuadtreeKey]: ...
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> Optional[QuadtreeKey]: ...
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int: ...
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey: ...
    def compute_common_ancestor_key(self, key1: QuadtreeKey, key2: QuadtreeKey) -> Tuple[QuadtreeKey, int]: ...
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> Optional[QuadtreeKey]: ...
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> Optional[QuadtreeKey]: ...
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> Optional[QuadtreeKey]: ...
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> Optional[QuadtreeKey]: ...
    @overload
    def key_to_coord(self, key: int) -> float: ...
    @overload
    def key_to_coord(self, key: int, depth: int) -> float: ...
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> Tuple[float, float]: ...
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> Tuple[float, float]: ...
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> Optional[QuadtreeKeyRay]: ...
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> Optional[list[Tuple[float, float]]]: ...
    def create_node_child(self, node: OccupancyQuadtreeNode, child_index: int) -> OccupancyQuadtreeNode: ...
    def delete_node_child(self, node: OccupancyQuadtreeNode, child_index: int) -> None: ...
    def get_node_child(self, node: OccupancyQuadtreeNode, child_index: int) -> OccupancyQuadtreeNode: ...
    def expand_node(self, node: OccupancyQuadtreeNode) -> None: ...
    def prune_node(self, node: OccupancyQuadtreeNode) -> None: ...
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> bool: ...
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> bool: ...
    def clear(self) -> None: ...
    def prune(self) -> None: ...
    def expand(self) -> None: ...
    @property
    def root(self) -> OccupancyQuadtreeNode: ...
    @overload
    def search(self, x: float, y: float) -> Optional[OccupancyQuadtreeNode]: ...
    @overload
    def search(self, key: QuadtreeKey) -> Optional[OccupancyQuadtreeNode]: ...
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> Optional[OccupancyQuadtreeNode]: ...
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> OccupancyQuadtreeNode: ...
    def visualize(
        self,
        leaf_only: bool = False,
        area_min: np.ndarray[np.float64] = None,
        area_max: np.ndarray[np.float64] = None,
        resolution: float = 0.1,
        padding: int = 1,
        bg_color: np.ndarray[np.float64] = np.array([128, 128, 128, 255]),
        fg_color: np.ndarray[np.float64] = np.array([255, 255, 255, 255]),
        occupied_color: np.ndarray[np.float64] = np.array([0, 0, 0, 255]),
        free_color: np.ndarray[np.float64] = np.array([255, 255, 255, 255]),
        border_color: np.ndarray[np.float64] = np.array([0, 0, 0, 255]),
        border_thickness: int = 1,
    ) -> np.ndarray[np.uint8]: ...

    class IteratorBase(collections.Iterable):
        def __iter__(self): ...
        def __next__(self): ...
        def __eq__(self, other) -> bool: ...
        def __ne__(self, other) -> bool: ...
        def get(self) -> OccupancyQuadtreeNode: ...
        @property
        def x(self) -> float: ...
        @property
        def y(self) -> float: ...
        @property
        def node_size(self) -> float: ...
        @property
        def node_aabb(self) -> Aabb2D: ...
        @property
        def key(self) -> QuadtreeKey: ...
        @property
        def index_key(self) -> QuadtreeKey: ...
        def next(self) -> None: ...
        @property
        def is_end(self) -> bool: ...

    class TreeIterator(IteratorBase): ...
    class TreeInAabbIterator(IteratorBase): ...
    class LeafIterator(IteratorBase): ...
    class LeafOfNodeIterator(IteratorBase): ...
    class LeafInAabbIterator(IteratorBase): ...
    class WestLeafNeighborIterator(IteratorBase): ...
    class EastLeafNeighborIterator(IteratorBase): ...
    class NorthLeafNeighborIterator(IteratorBase): ...
    class SouthLeafNeighborIterator(IteratorBase): ...
    class LeafOnRayIterator(IteratorBase): ...
    class OccupiedLeafOnRayIterator(IteratorBase): ...

    def iter_leaf(self, max_depth: int = 0) -> LeafIterator: ...
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = 0) -> LeafOfNodeIterator: ...
    @overload
    def iter_leaf_in_aabb(
        self,
        aabb_min_x: float,
        aabb_min_y: float,
        aabb_max_x: float,
        aabb_max_y: float,
        max_depth: int = 0,
    ) -> LeafInAabbIterator: ...
    @overload
    def iter_leaf_in_aabb(
        self,
        aabb_min_key: QuadtreeKey,
        aabb_max_key: QuadtreeKey,
        max_depth: int = 0,
    ) -> LeafInAabbIterator: ...
    def iter_node(self, max_depth: int = 0) -> TreeIterator: ...
    @overload
    def iter_node_in_aabb(
        self,
        aabb_min_x: float,
        aabb_min_y: float,
        aabb_max_x: float,
        aabb_max_y: float,
        max_depth: int = 0,
    ) -> TreeInAabbIterator: ...
    @overload
    def iter_node_in_aabb(
        self,
        aabb_min_key: QuadtreeKey,
        aabb_max_key: QuadtreeKey,
        max_depth: int = 0,
    ) -> TreeInAabbIterator: ...
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = 0) -> WestLeafNeighborIterator: ...
    @overload
    def iter_west_leaf_neighbor(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> WestLeafNeighborIterator: ...
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = 0) -> EastLeafNeighborIterator: ...
    @overload
    def iter_east_leaf_neighbor(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> EastLeafNeighborIterator: ...
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = 0) -> NorthLeafNeighborIterator: ...
    @overload
    def iter_north_leaf_neighbor(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> NorthLeafNeighborIterator: ...
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = 0) -> SouthLeafNeighborIterator: ...
    @overload
    def iter_south_leaf_neighbor(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> SouthLeafNeighborIterator: ...
    def iter_leaf_on_ray(
        self,
        px: float,
        py: float,
        vx: float,
        vy: float,
        max_range: float = -1,
        bidirectional: bool = False,
        max_leaf_depth: int = 0,
    ) -> LeafOnRayIterator: ...
    def iter_occupied_leaf_on_ray(
        self,
        px: float,
        py: float,
        vx: float,
        vy: float,
        max_range: float = -1,
        bidirectional: bool = False,
        max_leaf_depth: int = 0,
    ) -> OccupiedLeafOnRayIterator: ...

    class Drawer:
        class Setting(YamlableBase):
            area_min: np.ndarray[np.float64]
            area_max: np.ndarray[np.float64]
            resolution: float
            padding: int
            bg_color: np.ndarray[np.float64]
            fg_color: np.ndarray[np.float64]
            border_color: np.ndarray[np.float64]
            border_thickness: int
            occupied_color: np.ndarray[np.float64]
            free_color: np.ndarray[np.float64]

            def __init__(self: OccupancyQuadtree.Drawer.Setting) -> None: ...

        def __init__(self: OccupancyQuadtree.Drawer.Setting, quadtree: OccupancyQuadtree) -> None: ...
        @property
        def setting(self: OccupancyQuadtree.Drawer) -> OccupancyQuadtree.Drawer.Setting: ...
        @property
        def grid_map_info(self: OccupancyQuadtree.Drawer) -> GridMapInfo2D: ...
        def set_draw_tree_callback(self: OccupancyQuadtree.Drawer, callback: Callable) -> None: ...
        def set_draw_leaf_callback(self: OccupancyQuadtree.Drawer, callback: Callable) -> None: ...
        def draw_tree(self: OccupancyQuadtree.Drawer) -> np.ndarray[np.uint8]: ...
        def draw_leaves(self: OccupancyQuadtree.Drawer) -> np.ndarray[np.uint8]: ...

class OccupancyOctreeBaseSetting(OccupancyNdTreeSetting):
    use_change_detection: bool
    use_aabb_limit: bool
    aabb: Aabb3D

    def __init__(self: OccupancyOctreeBaseSetting) -> None: ...

class OctreeKey:
    def __eq__(self, other) -> bool: ...
    def __ne__(self, other) -> bool: ...
    def __getitem__(self, item) -> int: ...

class OctreeKeyRay:
    def __len__(self) -> int: ...
    def __getitem__(self, item) -> OctreeKey: ...

class OccupancyOctreeNode:
    @property
    def node_type(self) -> str: ...
    @property
    def depth(self) -> int: ...
    @property
    def child_index(self) -> int: ...
    @property
    def num_children(self) -> int: ...
    @property
    def has_any_child(self) -> bool: ...
    def has_child(self, child_index: int) -> bool: ...
    def get_child(self, child_index: int) -> OccupancyOctreeNode: ...
    @property
    def occupancy(self) -> float: ...
    @property
    def log_odds(self) -> float: ...
    @property
    def mean_child_log_odds(self) -> float: ...
    @property
    def max_child_log_odds(self) -> float: ...
    def allow_update_log_odds(self, delta: float) -> bool: ...
    def add_log_odds(self, log_odds: float) -> None: ...

class OccupancyOctree:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, setting: OccupancyOctreeBaseSetting) -> None: ...
    @overload
    def __init__(self, filename: str, is_binary: bool) -> None: ...
    def read_raw(self, filename: str) -> bool: ...
    def write_binary(self, filename: str, prune_at_first: bool) -> bool: ...
    def read_binary(self, filename: str) -> bool: ...
    def is_node_occupied(self, node: OccupancyOctreeNode) -> bool: ...
    @property
    def tree_type(self) -> str: ...
    @property
    def setting(self) -> OccupancyOctreeBaseSetting: ...
    @property
    def is_node_collapsible(self) -> bool: ...
    def insert_point_cloud(
        self,
        points: np.ndarray[np.float64],
        sensor_origin: np.ndarray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
        discretize: bool,
    ) -> None: ...
    def insert_point_cloud_rays(
        self,
        points: np.ndarray[np.float64],
        sensor_origin: np.ndarray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
    ) -> None: ...
    def insert_ray(
        self,
        sx: float,
        sy: float,
        sz: float,
        ex: float,
        ey: float,
        ez: float,
        max_range: float,
        lazy_eval: bool,
    ) -> None: ...
    def sample_positions(self, num_positions: int) -> list[np.ndarray]: ...
    @overload
    def cast_rays(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
        azimuth_angles: np.ndarray,
        elevation_angles: np.ndarray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": list[tuple[int, int]],
            "hit_positions": list[np.ndarray],
            "hit_nodes": list[OccupancyOctreeNode],
            "node_depths": list[int],
        },
    ): ...
    @overload
    def cast_rays(
        self,
        positions: np.ndarray,
        directions: np.ndarray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": list[int],
            "hit_positions": np.ndarray,
            "hit_nodes": list[OccupancyOctreeNode],
            "node_depths": list[int],
        },
    ): ...
    def cast_ray(
        self,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        ignore_unknown: bool,
        max_range: float,
    ) -> TypedDict("returns", {"hit_node": OccupancyQuadtreeNode, "ex": float, "ey": float}): ...
    @overload
    def update_node(self, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> OccupancyOctreeNode: ...
    @overload
    def update_node(self, node_key: OctreeKey, occupied: bool, lazy_eval: bool) -> OccupancyOctreeNode: ...
    @overload
    def update_node(
        self,
        x: float,
        y: float,
        z: float,
        log_odds_delta: float,
        lazy_eval: bool,
    ) -> OccupancyOctreeNode: ...
    @overload
    def update_node(self, node_key: OctreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyOctreeNode: ...
    def update_inner_occupancy(self) -> None: ...
    def to_max_likelihood(self) -> None: ...
    @property
    def number_of_nodes(self) -> int: ...

    resolution: float

    @property
    def tree_depth(self) -> int: ...
    @property
    def tree_key_offset(self) -> int: ...
    @property
    def metric_min(self) -> Tuple[float, float, float]: ...
    @property
    def metric_max(self) -> Tuple[float, float, float]: ...
    @property
    def metric_min_max(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]: ...
    @property
    def metric_size(self) -> Tuple[float, float, float]: ...
    def get_node_size(self, depth: int) -> float: ...
    @property
    def number_of_leaf_nodes(self) -> int: ...
    @property
    def memory_usage(self) -> int: ...
    @property
    def memory_usage_per_node(self) -> int: ...
    @overload
    def coord_to_key(self, coordinate: float) -> int: ...
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int: ...
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey: ...
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey: ...
    @overload
    def coord_to_key_checked(self, coordinate: float) -> Optional[int]: ...
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> Optional[int]: ...
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> Optional[OctreeKey]: ...
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> Optional[OctreeKey]: ...
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int: ...
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey: ...
    def compute_common_ancestor_key(self, key1: OctreeKey, key2: OctreeKey) -> Tuple[OctreeKey, int]: ...
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> Optional[OctreeKey]: ...
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> Optional[OctreeKey]: ...
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> Optional[OctreeKey]: ...
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> Optional[OctreeKey]: ...
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> Optional[OctreeKey]: ...
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> Optional[OctreeKey]: ...
    @overload
    def key_to_coord(self, key: int) -> float: ...
    @overload
    def key_to_coord(self, key: int, depth: int) -> float: ...
    @overload
    def key_to_coord(self, key: OctreeKey) -> Tuple[float, float, float]: ...
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> Tuple[float, float, float]: ...
    def compute_ray_keys(
        self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float
    ) -> Optional[OctreeKeyRay]: ...
    def compute_ray_coords(
        self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float
    ) -> Optional[list[Tuple[float, float, float]]]: ...
    def create_node_child(self, node: OccupancyOctreeNode, child_index: int) -> OccupancyOctreeNode: ...
    def delete_node_child(self, node: OccupancyOctreeNode, child_index: int) -> None: ...
    def get_node_child(self, node: OccupancyOctreeNode, child_index: int) -> OccupancyOctreeNode: ...
    def expand_node(self, node: OccupancyOctreeNode) -> None: ...
    def prune_node(self, node: OccupancyOctreeNode) -> None: ...
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> bool: ...
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> bool: ...
    def clear(self) -> None: ...
    def prune(self) -> None: ...
    def expand(self) -> None: ...
    @property
    def root(self) -> OccupancyOctreeNode: ...
    @overload
    def search(self, x: float, y: float, z: float) -> Optional[OccupancyOctreeNode]: ...
    @overload
    def search(self, key: OctreeKey) -> Optional[OccupancyOctreeNode]: ...
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> Optional[OccupancyOctreeNode]: ...
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> OccupancyOctreeNode: ...
    def visualize(
        self,
        leaf_only: bool = False,
        occupied_color: np.ndarray[np.float64] = np.array([0.5, 0.5, 0.5]),
        border_color: np.ndarray[np.float64] = np.array([0.0, 0.0, 0.0]),
        window_width: int = 1920,
        window_height: int = 1080,
        window_left: int = 50,
        window_top: int = 50,
    ) -> np.ndarray[np.uint8]: ...

    class IteratorBase(collections.Iterable):
        def __iter__(self): ...
        def __next__(self): ...
        def __eq__(self, other) -> bool: ...
        def __ne__(self, other) -> bool: ...
        def get(self) -> OccupancyOctreeNode: ...
        @property
        def x(self) -> float: ...
        @property
        def y(self) -> float: ...
        @property
        def z(self) -> float: ...
        @property
        def node_size(self) -> float: ...
        @property
        def node_aabb(self) -> Aabb3D: ...
        @property
        def key(self) -> OctreeKey: ...
        @property
        def index_key(self) -> OctreeKey: ...
        def next(self) -> None: ...
        @property
        def is_end(self) -> bool: ...

    class TreeIterator(IteratorBase): ...
    class TreeInAabbIterator(IteratorBase): ...
    class LeafIterator(IteratorBase): ...
    class LeafOfNodeIterator(IteratorBase): ...
    class LeafInAabbIterator(IteratorBase): ...
    class WestLeafNeighborIterator(IteratorBase): ...
    class EastLeafNeighborIterator(IteratorBase): ...
    class NorthLeafNeighborIterator(IteratorBase): ...
    class SouthLeafNeighborIterator(IteratorBase): ...
    class TopLeafNeighborIterator(IteratorBase): ...
    class BottomLeafNeighborIterator(IteratorBase): ...
    class LeafOnRayIterator(IteratorBase): ...
    class OccupiedLeafOnRayIterator(IteratorBase): ...

    def iter_leaf(self, max_depth: int = 0) -> LeafIterator: ...
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = 0) -> LeafOfNodeIterator: ...
    @overload
    def iter_leaf_in_aabb(
        self,
        aabb_min_x: float,
        aabb_min_y: float,
        aabb_min_z: float,
        aabb_max_x: float,
        aabb_max_y: float,
        aabb_max_z: float,
        max_depth: int = 0,
    ) -> LeafInAabbIterator: ...
    @overload
    def iter_leaf_in_aabb(
        self,
        aabb_min_key: OctreeKey,
        aabb_max_key: OctreeKey,
        max_depth: int = 0,
    ) -> LeafInAabbIterator: ...
    def iter_node(self, max_depth: int = 0) -> TreeIterator: ...
    @overload
    def iter_node_in_aabb(
        self,
        aabb_min_x: float,
        aabb_min_y: float,
        aabb_min_z: float,
        aabb_max_x: float,
        aabb_max_y: float,
        aabb_max_z: float,
        max_depth: int = 0,
    ) -> TreeInAabbIterator: ...
    @overload
    def iter_node_in_aabb(
        self,
        aabb_min_key: OctreeKey,
        aabb_max_key: OctreeKey,
        max_depth: int = 0,
    ) -> TreeInAabbIterator: ...
    @overload
    def iter_west_leaf_neighbor(
        self,
        x: float,
        y: float,
        z: float,
        max_leaf_depth: int = 0,
    ) -> WestLeafNeighborIterator: ...
    @overload
    def iter_west_leaf_neighbor(
        self,
        key: OctreeKey,
        key_depth: int,
        max_leaf_depth: int = 0,
    ) -> WestLeafNeighborIterator: ...
    @overload
    def iter_east_leaf_neighbor(
        self,
        x: float,
        y: float,
        z: float,
        max_leaf_depth: int = 0,
    ) -> EastLeafNeighborIterator: ...
    @overload
    def iter_east_leaf_neighbor(
        self,
        key: OctreeKey,
        key_depth: int,
        max_leaf_depth: int = 0,
    ) -> EastLeafNeighborIterator: ...
    @overload
    def iter_north_leaf_neighbor(
        self,
        x: float,
        y: float,
        z: float,
        max_leaf_depth: int = 0,
    ) -> NorthLeafNeighborIterator: ...
    @overload
    def iter_north_leaf_neighbor(
        self,
        key: OctreeKey,
        key_depth: int,
        max_leaf_depth: int = 0,
    ) -> NorthLeafNeighborIterator: ...
    @overload
    def iter_south_leaf_neighbor(
        self,
        x: float,
        y: float,
        z: float,
        max_leaf_depth: int = 0,
    ) -> SouthLeafNeighborIterator: ...
    @overload
    def iter_south_leaf_neighbor(
        self,
        key: OctreeKey,
        key_depth: int,
        max_leaf_depth: int = 0,
    ) -> SouthLeafNeighborIterator: ...
    @overload
    def iter_top_leaf_neighbor(
        self,
        x: float,
        y: float,
        z: float,
        max_leaf_depth: int = 0,
    ) -> TopLeafNeighborIterator: ...
    @overload
    def iter_top_leaf_neighbor(
        self,
        key: OctreeKey,
        key_depth: int,
        max_leaf_depth: int = 0,
    ) -> TopLeafNeighborIterator: ...
    @overload
    def iter_bottom_leaf_neighbor(
        self,
        x: float,
        y: float,
        z: float,
        max_leaf_depth: int = 0,
    ) -> BottomLeafNeighborIterator: ...
    @overload
    def iter_bottom_leaf_neighbor(
        self,
        key: OctreeKey,
        key_depth: int,
        max_leaf_depth: int = 0,
    ) -> BottomLeafNeighborIterator: ...
    def iter_leaf_on_ray(
        self,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        max_range: float = -1,
        bidirectional: bool = False,
        max_leaf_depth: int = 0,
    ) -> LeafOnRayIterator: ...
    def iter_occupied_leaf_on_ray(
        self,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        max_range: float = -1,
        bidirectional: bool = False,
        max_leaf_depth: int = 0,
    ) -> OccupiedLeafOnRayIterator: ...

    class Drawer:
        class Setting(YamlableBase):
            area_min: np.ndarray[np.float64]
            area_max: np.ndarray[np.float64]
            occupied_only: bool
            occupied_color: np.ndarray[np.float64]
            draw_node_boxes: bool
            draw_node_borders: bool

            def __init__(self: OccupancyOctree.Drawer.Setting) -> None: ...

        def __init__(self: OccupancyOctree.Drawer.Setting, octree: OccupancyOctree) -> None: ...
        @property
        def setting(self: OccupancyOctree.Drawer) -> OccupancyOctree.Drawer.Setting: ...
        @property
        def grid_map_info(self: OccupancyOctree.Drawer) -> GridMapInfo2D: ...
        def set_draw_tree_callback(self: OccupancyOctree.Drawer, callback: Callable) -> None: ...
        def set_draw_leaf_callback(self: OccupancyOctree.Drawer, callback: Callable) -> None: ...
        def draw_tree(self: OccupancyOctree.Drawer) -> np.ndarray[np.uint8]: ...
        def draw_leaves(self: OccupancyOctree.Drawer) -> np.ndarray[np.uint8]: ...

class Surface2D:
    @overload
    def __init__(
        self: Surface2D,
        vertices: np.ndarray[np.float64],
        normals: np.ndarray[np.float64],
        lines2vertices: np.ndarray[np.float64],
        objects2lines: np.ndarray[np.float64],
        outside_flags: np.ndarray[np.bool_],
    ): ...
    @overload
    def __init__(self: Surface2D): ...
    @property
    def num_vertices(self: Surface2D) -> int: ...
    @property
    def num_lines(self: Surface2D) -> int: ...
    @property
    def num_objects(self: Surface2D) -> int: ...
    @property
    def vertices(self: Surface2D) -> np.ndarray[np.float64]: ...
    @property
    def normals(self: Surface2D) -> np.ndarray[np.float64]: ...
    @property
    def lines_to_vertices(self: Surface2D) -> np.ndarray[np.float64]: ...
    @property
    def objects_to_lines(self: Surface2D) -> np.ndarray[np.float64]: ...
    @property
    def vertices_to_objects(self: Surface2D) -> np.ndarray[np.float64]: ...
    @property
    def outside_flags(self: Surface2D) -> np.ndarray[np.bool_]: ...
    @property
    def normals_available(self: Surface2D) -> bool: ...
    @property
    def outside_flags_available(self: Surface2D) -> bool: ...
    def get_object_vertices(self: Surface2D, index_object: int) -> np.ndarray[np.float64]: ...
    def get_object_normals(self: Surface2D, index_object: int) -> np.ndarray[np.float64]: ...
    def get_vertex_neighbors(self: Surface2D, index_vertex: int) -> np.ndarray[np.float64]: ...

class Space2D:
    class SignMethod(IntEnum):
        kPointNormal = 0
        kLineNormal = 1
        kPolygon = 2

    @overload
    def __init__(
        self: Space2D,
        ordered_object_vertices: list[np.ndarray[np.float64]],
        ordered_object_normals: list[np.ndarray[np.float64]],
    ): ...
    @overload
    def __init__(
        self: Space2D,
        ordered_object_vertices: list[np.ndarray[np.float64]],
        outside_flags: np.ndarray[np.bool_],
        delta: float = 0.01,
        parallel: bool = False,
    ): ...
    @overload
    def __init__(
        self: Space2D,
        map_image: np.ndarray[np.float64],
        grid_map_info: GridMapInfo2D,
        free_threshold: float,
        delta: float = 0.01,
        parallel: bool = False,
    ): ...
    @overload
    def __init__(self: Space2D, space2d: Space2D): ...
    @property
    def surface(self: Space2D) -> Surface2D: ...
    def get_sign_method_name(self: Space2D, sign_method: SignMethod) -> str: ...
    def get_sign_method_from_name(self: Space2D, sign_method_name: str) -> SignMethod: ...
    def generate_map_image(self: Space2D, map_image_info: GridMapInfo2D) -> np.ndarray[np.uint8]: ...
    @overload
    def compute_sdf_image(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        sign_method: SignMethod = SignMethod.kLineNormal,
        use_kdtree: bool = False,
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    @overload
    def compute_sdf(
        self: Space2D,
        query_points: np.ndarray[np.float64],
        sign_method: SignMethod = SignMethod.kLineNormal,
        use_kdtree: bool = False,
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    def compute_sdf_with_kdtree(self: Space2D, q: np.ndarray[np.float64], sign_method: SignMethod) -> float: ...
    def compute_sdf_greedily(self: Space2D, q: np.ndarray[np.float64], sign_method: SignMethod) -> float: ...
    @overload
    def compute_ddf(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        query_directions: np.ndarray[np.float64],
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    @overload
    def compute_ddf(
        self: Space2D,
        query_points: np.ndarray[np.float64],
        query_directions: np.ndarray[np.float64],
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    @overload
    def compute_sddf_v1(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        query_directions: np.ndarray[np.float64],
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    @overload
    def compute_sddf_v1(
        self: Space2D,
        query_points: np.ndarray[np.float64],
        query_directions: np.ndarray[np.float64],
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    @overload
    def compute_sddf_v2(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        query_directions: np.ndarray[np.float64],
        sign_method: SignMethod = SignMethod.kLineNormal,
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...
    @overload
    def compute_sddf_v2(
        self: Space2D,
        query_points: np.ndarray[np.float64],
        query_directions: np.ndarray[np.float64],
        sign_method: SignMethod = SignMethod.kLineNormal,
        parallel: bool = False,
    ) -> np.ndarray[np.float64]: ...

class Lidar2D:
    class Mode(IntEnum):
        kDdf = 0
        kSddfV1 = 1
        kSddfV2 = 2

    class Setting(YamlableBase):
        min_angle: float
        max_angle: float
        num_lines: int
        mode: Lidar2D.Mode
        sign_method: Space2D.SignMethod

    def __init__(self: Lidar2D, setting: Lidar2D.Setting, space2d: Space2D): ...
    @property
    def setting(self: Lidar2D) -> Lidar2D.Setting: ...
    @property
    def angles(self: Lidar2D) -> np.ndarray: ...
    @property
    def ray_directions_in_frame(self: Lidar2D) -> np.ndarray: ...
    @overload
    def scan(self: Lidar2D, rotation_angle: float, translation: np.ndarray, parallel: bool = False) -> np.ndarray: ...
    @overload
    def scan(self: Lidar2D, rotation: np.ndarray, translation: np.ndarray, parallel: bool = False) -> np.ndarray: ...
    def scan_multi_poses(self: Lidar2D, poses: list[np.ndarray], parallel: bool = False) -> np.ndarray: ...

class LidarFramePartition2D:
    @property
    def index_begin(self) -> int: ...
    @property
    def index_end(self) -> int: ...
    def angle_in_partition(self, angle_world: float) -> bool: ...

class LidarFrame2D:
    class Setting(YamlableBase):
        valid_range_min: float
        valid_range_max: float
        valid_angle_min: float
        valid_angle_max: float
        discontinuity_factor: float
        rolling_diff_discount: float
        min_partition_size: int

    def __init__(self: LidarFrame2D, setting: LidarFrame2D.Setting) -> None: ...
    def update(
        self,
        rotation: np.ndarray[np.float64],
        translation: np.ndarray[np.float64],
        angles: np.ndarray[np.float64],
        ranges: np.ndarray[np.float64],
        partition_rays: bool = False,
    ): ...
    @property
    def setting(self) -> LidarFrame2D.Setting: ...
    @property
    def num_rays(self) -> int: ...
    @property
    def num_hit_rays(self) -> int: ...
    @property
    def rotation_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def rotation_angle(self) -> float: ...
    @property
    def translation_vector(self) -> np.ndarray[np.float64]: ...
    @property
    def pose_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def angles_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def angles_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def ranges(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def end_points_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def end_points_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def hit_ray_indices(self) -> np.ndarray[np.int64]: ...
    @property
    def hit_points_world(self) -> np.ndarray[np.float64]: ...
    @property
    def max_valid_range(self) -> float: ...
    @property
    def partitions(self) -> list[LidarFramePartition2D]: ...
    @property
    def is_valid(self) -> bool: ...
    def compute_closest_end_point(
        self, position: np.ndarray[np.float64]
    ) -> TypedDict("returns", {"end_point_index": int, "distance": float}): ...
    @overload
    def sample_along_rays(
        self,
        num_samples_per_ray: int,
        max_in_obstacle_dist: float,
        sampled_rays_ratio: float,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    @overload
    def sample_along_rays(
        self,
        range_step: float,
        max_in_obstacle_dist: float,
        sampled_rays_ratio: float,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_near_surface(
        self,
        num_samples_per_ray: int,
        max_offset: float,
        sampled_rays_ratio: float,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_in_region(
        self,
        num_positions: int,
        num_along_ray_samples_per_ray: int,
        num_near_surface_samples_per_ray: int,
        max_in_obstacle_dist: float,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def compute_rays_at(
        self, position_world: np.ndarray[np.float64]
    ) -> TypedDict(
        "returns",
        {
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
            "visible_hit_point_indices": list[int],
        },
    ): ...

class LogOddMap2D:
    class CellType(IntEnum):
        kOccupied = 0
        kFree = 255
        kUnexplored = 128

    class Setting(YamlableBase):
        sensor_min_range: float
        sensor_max_range: float
        measurement_certainty: float
        max_log_odd: float
        min_log_odd: float
        threshold_occupied: float
        threshold_free: float
        use_cross_kernel: bool
        num_iters_for_cleaned_mask: int
        filter_obstacles_in_cleaned_mask: bool

    @overload
    def __init__(self: LogOddMap2D, setting: Setting, grid_map_info: GridMapInfo2D) -> None: ...
    @overload
    def __init__(
        self: LogOddMap2D,
        setting: Setting,
        grid_map_info: GridMapInfo2D,
        shape_metric_vertices: np.ndarray[np.float64],
    ) -> None: ...
    def update(
        self: LogOddMap2D,
        position: np.ndarray[np.float64],
        theta: float,
        angles_body: np.ndarray[np.float64],
        ranges: np.ndarray[np.float64],
    ) -> None: ...
    def compute_statistics_of_lidar_frame(
        self: LogOddMap2D,
        position: np.ndarray[np.float64],
        theta: float,
        angles_body: np.ndarray[np.float64],
        ranges: np.ndarray[np.float64],
        clip_ranges: bool,
    ) -> Tuple[int, int, int, int]: ...
    @property
    def setting(self: LogOddMap2D) -> Setting: ...
    @property
    def log_map(self: LogOddMap2D) -> np.ndarray[np.float64]: ...
    @property
    def possibility_map(self: LogOddMap2D) -> np.ndarray[np.float64]: ...
    @property
    def occupancy_map(self: LogOddMap2D) -> np.ndarray[np.uint8]: ...
    @property
    def unexplored_mask(self: LogOddMap2D) -> np.ndarray[np.uint8]: ...
    @property
    def occupied_mask(self: LogOddMap2D) -> np.ndarray[np.unit8]: ...
    @property
    def free_mask(self: LogOddMap2D) -> np.ndarray[np.uint8]: ...
    @property
    def num_unexplored_cells(self: LogOddMap2D) -> int: ...
    @property
    def num_occupied_cells(self: LogOddMap2D) -> int: ...
    @property
    def num_free_cells(self: LogOddMap2D) -> int: ...
    def get_frontiers(
        self: LogOddMap2D, clean_at_first: bool = True, approx_iters: int = 4
    ) -> list[np.ndarray[np.int32]]: ...

class Lidar3D:
    class Setting(YamlableBase):
        azimuth_min: float
        azimuth_max: float
        elevation_min: float
        elevation_max: float
        num_azimuth_lines: int
        num_elevation_lines: int

    @overload
    def __init__(self, setting: Setting, vertices: np.ndarray, triangles: np.ndarray) -> None: ...
    @overload
    def __init__(self, setting: Setting, vertices: list[np.ndarray], triangles: list[np.ndarray]) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def azimuth_angles(self) -> np.ndarray[np.float64]: ...
    @property
    def elevation_angles(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> np.ndarray[np.float64]: ...
    def scan(
        self,
        orientation: np.ndarray[np.float64],
        translation: np.ndarray[np.float64],
        add_noise: bool = False,
        noise_stddev: float = 0.03,
    ) -> np.ndarray[np.float64]: ...

class DepthCamera3D:
    class Setting(YamlableBase):
        image_height: int
        image_width: int
        camera_fx: float
        camera_fy: float
        camera_cx: float
        camera_cy: float

    @overload
    def __init__(self, setting: Setting, vertices: np.ndarray, triangles: np.ndarray) -> None: ...
    @overload
    def __init__(self, setting: Setting, vertices: list[np.ndarray], triangles: list[np.ndarray]) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def ray_directions_in_frame(self) -> np.ndarray[np.float64]: ...
    def scan(
        self,
        rotation: np.ndarray[np.float64],
        translation: np.ndarray[np.float64],
        add_noise: bool = False,
        noise_stddev: float = 0.03,
    ) -> np.ndarray[np.float64]: ...

class LidarFramePartition3D: ...

class LidarFrame3D:
    class Setting(YamlableBase):
        valid_range_min: float
        valid_range_max: float
        valid_azimuth_min: float
        valid_azimuth_max: float
        valid_elevation_min: float
        valid_elevation_max: float
        discontinuity_factor: float
        rolling_diff_discount: float
        min_partition_size: int

    def __init__(self: LidarFrame3D, setting: Setting) -> None: ...
    def reset(self) -> None: ...
    def update(
        self,
        rotation: np.ndarray[np.float64],
        translation: np.ndarray[np.float64],
        azimuths: np.ndarray[np.float64],
        elevations: np.ndarray[np.float64],
        ranges: np.ndarray[np.float64],
        partition_rays: bool = False,
    ) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def num_rays(self) -> int: ...
    @property
    def num_hit_rays(self) -> int: ...
    @property
    def rotation_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def translation_vector(self) -> np.ndarray[np.float64]: ...
    @property
    def pose_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def azimuth_angles_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def elevation_angles_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def ranges(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def end_points_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def end_points_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def hit_ray_indices(self) -> np.ndarray[np.int64]: ...
    @property
    def hit_points_world(self) -> np.ndarray[np.float64]: ...
    @property
    def max_valid_range(self) -> float: ...
    @property
    def is_valid(self) -> bool: ...
    @property
    def is_partitioned(self) -> bool: ...
    @property
    def partitions(self) -> list[LidarFramePartition3D]: ...
    def compute_closest_end_point(
        self, position_world: np.ndarray[np.float64]
    ) -> TypedDict("returns", {"azimuth_index": int, "elevation_index": int, "distance": float}): ...
    @overload
    def sample_along_rays(
        self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    @overload
    def sample_along_rays(
        self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_near_surface(
        self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_in_region_hpr(
        self,
        num_positions: int,
        num_near_surface_samples_per_ray: int,
        num_along_ray_samples_per_ray: int,
        max_in_obstacle_dist: float,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_in_region_vrs(
        self,
        num_hit_points: int,
        num_samples_per_azimuth_segment: int,
        num_azimuth_segments: int,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def compute_rays_at(
        self, position_world: np.ndarray[np.float64]
    ) -> TypedDict(
        "returns",
        {
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
            "visible_hit_point_indices": list[int],
        },
    ): ...

class RgbdFrame3D:
    class Setting(LidarFrame3D.Setting):
        image_height: int
        image_width: int
        camera_fx: float
        camera_fy: float
        camera_cx: float
        camera_cy: float
        depth_scale: float

    def __init__(self: RgbdFrame3D, setting: Setting) -> None: ...
    def reset(self) -> None: ...
    def resize(self, factor: float) -> Tuple[int, int]: ...
    @overload
    def update(
        self,
        orientation: np.ndarray[np.float64],
        translation: np.ndarray[np.float64],
        depth: np.ndarray[np.float64],
        depth_scaled: bool = True,
        partition_rays: bool = False,
    ) -> None: ...
    @overload
    def update(
        self,
        orientation: np.ndarray[np.float64],
        translation: np.ndarray[np.float64],
        depth_file: str,
        partition_rays: bool = False,
    ) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def camera_extrinsic_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def camera_intrinsic_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def num_rays(self) -> int: ...
    @property
    def num_hit_rays(self) -> int: ...
    @property
    def rotation_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def translation_vector(self) -> np.ndarray[np.float64]: ...
    @property
    def pose_matrix(self) -> np.ndarray[np.float64]: ...
    @property
    def azimuth_angles_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def elevation_angles_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def ranges(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def ray_directions_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def end_points_in_frame(self) -> np.ndarray[np.float64]: ...
    @property
    def end_points_in_world(self) -> np.ndarray[np.float64]: ...
    @property
    def hit_ray_indices(self) -> np.ndarray[np.int64]: ...
    @property
    def hit_points_world(self) -> np.ndarray[np.float64]: ...
    @property
    def max_valid_range(self) -> float: ...
    @property
    def is_valid(self) -> bool: ...
    @property
    def is_partitioned(self) -> bool: ...
    @property
    def partitions(self) -> list[LidarFramePartition3D]: ...
    def compute_closest_end_point(
        self, position_world: np.ndarray[np.float64]
    ) -> TypedDict("returns", {"azimuth_index": int, "elevation_index": int, "distance": float}): ...
    @overload
    def sample_along_rays(
        self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    @overload
    def sample_along_rays(
        self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_near_surface(
        self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_in_region_hpr(
        self,
        num_positions: int,
        num_near_surface_samples_per_ray: int,
        num_along_ray_samples_per_ray: int,
        max_in_obstacle_dist: float,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def sample_in_region_vrs(
        self,
        num_hit_points: int,
        num_samples_per_azimuth_segment: int,
        num_azimuth_segments: int,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "positions_world": np.ndarray[np.float64],
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
        },
    ): ...
    def compute_rays_at(
        self, position_world: np.ndarray[np.float64]
    ) -> TypedDict(
        "returns",
        {
            "directions_world": np.ndarray[np.float64],
            "distances": np.ndarray[np.float64],
            "visible_hit_point_indices": list[int],
        },
    ): ...
