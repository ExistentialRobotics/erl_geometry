import collections
from enum import IntEnum
from typing import Callable
from typing import Optional
from typing import Tuple
from typing import TypedDict
from typing import overload

import numpy as np
import numpy.typing as npt
from erl_common.storage import GridMapInfo2D
from erl_common.yaml import YamlableBase

__all__ = [
    "marching_square",
    "bresenham_2d",
    "compute_pixel_of_polygon_contour",
    "winding_number",
    "compute_nearest_distance_from_point_to_line_segment_2d",
    "compute_intersection_between_ray_and_line_2d",
    "compute_intersection_between_ray_and_aabb_2d",
    "compute_intersection_between_ray_and_aabb_3d",
    "convert_path_2d_to_3d",
    "Aabb2D",
    "Aabb3D",
    "CityStreetMap",
    "NdTreeSetting",
    "OccupancyNdTreeSetting",
    "OccupancyQuadtreeBaseSetting",
    "OccupancyOctreeBaseSetting",
    "QuadtreeKey",
    "QuadtreeKeyRay",
    "AbstractQuadtreeNode",
    "OccupancyQuadtreeNode",
    "PyObjectOccupancyQuadtreeNode",
    "SurfaceMappingQuadtreeNode",
    "AbstractQuadtree",
    "AbstractOccupancyQuadtree",
    "OccupancyQuadtree",
    "PyObjectOccupancyQuadtree",
    "SurfaceMappingQuadtree",
    "OctreeKey",
    "OctreeKeyRay",
    "AbstractOctreeNode",
    "OccupancyOctreeNode",
    "PyObjectOccupancyOctreeNode",
    "SurfaceMappingOctreeNode",
    "AbstractOctree",
    "AbstractOccupancyOctree",
    "OccupancyOctree",
    "PyObjectOccupancyOctree",
    "SurfaceMappingOctree",
    "Surface2D",
    "Space2D",
    "Lidar2D",
    "LidarFramePartition2D",
    "LidarFrame2D",
    "LogOddMap2D",
    "Lidar3D",
    "DepthCamera3D",
    "LidarFramePartition3D",
    "RangeSensorFrame3D",
    "LidarFrame3D",
    "DepthFrame3D",
    "AbstractSurfaceMapping",
    "AbstractSurfaceMapping2D",
    "AbstractSurfaceMapping3D",
    "Primitive2D",
    "Line2D",
    "Segment2D",
    "Ray2D",
    "AxisAlignedRectangle2D",
    "Rectangle2D",
    "Ellipse2D",
    "Primitive3D",
    "Ellipsoid",
    "Trajectory",
]

def marching_square(
    img: npt.NDArray[np.float64], iso_value: float
) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]: ...
def bresenham_2d(
    start: npt.NDArray[np.float64], end: npt.NDArray[np.float64], stop: Callable[[int, int], bool] = None
) -> npt.NDArray[np.float64]: ...
def compute_pixel_of_polygon_contour(polygon_vertices: npt.NDArray[np.int32]) -> npt.NDArray[np.int32]: ...
def winding_number(p: npt.NDArray[np.float64], vertices: npt.NDArray[np.float64]) -> int: ...
def compute_nearest_distance_from_point_to_line_segment_2d(
    point_x: float,
    point_y: float,
    line_segment_x1: float,
    line_segment_y1: float,
    line_segment_x2: float,
    line_segment_y2: float,
) -> float: ...
def compute_intersection_between_ray_and_line_2d(
    ray_start_point: npt.NDArray[np.float64],
    ray_direction: npt.NDArray[np.float64],
    segment_point1: npt.NDArray[np.float64],
    segment_point2: npt.NDArray[np.float64],
) -> Tuple[float, float, bool]:
    """
    Returns:
        Tuple[float, float, bool]: (t, distance, intersected) where
        intersection = t * segment_point1 + (1 - t) * segment_point2 when intersected is True.
    """
    ...

def compute_intersection_between_ray_and_aabb_2d(
    ray_start_point: npt.NDArray[np.float64],
    ray_direction: npt.NDArray[np.float64],
    aabb_min: npt.NDArray[np.float64],
    aabb_max: npt.NDArray[np.float64],
) -> TypedDict("returns", {"d1": float, "d2": float, "intersected": bool, "is_inside": bool}): ...
def compute_intersection_between_ray_and_aabb_3d(
    ray_start_point: npt.NDArray[np.float64],
    ray_direction: npt.NDArray[np.float64],
    aabb_min: npt.NDArray[np.float64],
    aabb_max: npt.NDArray[np.float64],
) -> TypedDict("returns", {"d1": float, "d2": float, "intersected": bool, "is_inside": bool}): ...
def compute_intersection_between_line_and_ellipse_2d(
    x0: float, y0: float, x1: float, y1: float, a: float, b: float
) -> Tuple[float, float, bool]: ...
def compute_intersection_between_ray_and_ellipsoid_3d(
    x0: float, y0: float, z0: float, x1: float, y1: float, z1: float, a: float, b: float, c: float
) -> Tuple[float, float, bool]: ...
def convert_path_2d_to_3d(path_2d: npt.NDArray, z: float) -> list[npt.NDArray]: ...

class Aabb2D:
    class CornerType(IntEnum):
        kBottomLeft = 0
        kBottomRight = 1
        kTopLeft = 2
        kTopRight = 3

    def __init__(self: Aabb2D, center: npt.NDArray, half_size: float): ...
    @property
    def center(self: Aabb2D) -> npt.NDArray: ...
    @property
    def half_sizes(self: Aabb2D) -> float: ...
    @property
    def min(self: Aabb2D) -> npt.NDArray: ...
    @property
    def max(self: Aabb2D) -> npt.NDArray: ...
    @overload
    def __contains__(self, point: npt.NDArray) -> bool: ...
    @overload
    def __contains__(self: Aabb2D, aabb: Aabb2D) -> bool: ...
    @overload
    def padding(self: Aabb2D, padding: npt.NDArray) -> Aabb2D: ...
    @overload
    def padding(self: Aabb2D, padding: float) -> Aabb2D: ...
    def corner(self: Aabb2D, corner_type: int) -> npt.NDArray: ...
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

    def __init__(self: Aabb3D, center: npt.NDArray, half_size: float): ...
    @property
    def center(self: Aabb3D) -> npt.NDArray: ...
    @property
    def half_sizes(self: Aabb3D) -> float: ...
    @property
    def min(self: Aabb3D) -> npt.NDArray: ...
    @property
    def max(self: Aabb3D) -> npt.NDArray: ...
    @overload
    def __contains__(self, point: npt.NDArray) -> bool: ...
    @overload
    def __contains__(self: Aabb3D, aabb: Aabb3D) -> bool: ...
    @overload
    def padding(self: Aabb3D, padding: npt.NDArray) -> Aabb3D: ...
    @overload
    def padding(self: Aabb3D, padding: float) -> Aabb3D: ...
    def corner(self: Aabb3D, corner_type: int) -> npt.NDArray: ...
    def intersects(self: Aabb3D, aabb: Aabb3D) -> bool: ...

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
    def load_map(filename: str) -> npt.NDArray: ...

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

class AbstractQuadtreeNode:
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

class AbstractQuadtree:
    def apply_setting(self) -> None: ...
    def write_raw(self, filename: str) -> bool: ...
    def read_raw(self, filename: str) -> bool: ...
    def search_node(self, x: float, y: float, max_depth: int) -> Optional[AbstractQuadtreeNode]: ...

    class QuadtreeNodeIterator:
        @property
        def x(self) -> float: ...
        @property
        def y(self) -> float: ...
        @property
        def node_size(self) -> float: ...
        @property
        def depth(self) -> int: ...
        def next(self) -> None: ...
        @property
        def is_valid(self) -> bool: ...

class AbstractOctreeNode:
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

class AbstractOctree:
    def apply_setting(self) -> None: ...
    def write_raw(self, filename: str) -> bool: ...
    def read_raw(self, filename: str) -> bool: ...
    def search_node(self, x: float, y: float, z: float, max_depth: int) -> Optional[AbstractOctreeNode]: ...

    class OctreeNodeIterator:
        @property
        def x(self) -> float: ...
        @property
        def y(self) -> float: ...
        @property
        def z(self) -> float: ...
        @property
        def node_size(self) -> float: ...
        @property
        def depth(self) -> int: ...
        def next(self) -> None: ...
        @property
        def is_valid(self) -> bool: ...

    def get_leaf_iterator(self) -> OctreeNodeIterator: ...
    def get_leaf_in_aabb_iterator(self, aabb: Aabb3D, max_depth: int) -> OctreeNodeIterator: ...
    def get_tree_iterator(self, max_depth: int) -> OctreeNodeIterator: ...
    def get_tree_in_aabb_iterator(self, aabb: Aabb3D, max_depth: int) -> OctreeNodeIterator: ...
    def get_node_on_ray_iterator(
        self,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        max_range: float,
        node_padding: float,
        bidirectional: bool,
        leaf_only: bool,
        min_node_depth: int,
        max_node_depth: int,
    ) -> OctreeNodeIterator: ...

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
    def __hash__(self) -> int: ...

class QuadtreeKeyRay:
    def __len__(self) -> int: ...
    def __getitem__(self, item) -> QuadtreeKey: ...

class OccupancyQuadtreeNode(AbstractQuadtreeNode):
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

class AbstractOccupancyQuadtree(AbstractQuadtree):
    def write_binary(self, filename: str, prune_at_first: bool) -> bool: ...
    def read_binary(self, filename: str) -> bool: ...
    def is_node_occupied(self, node: OccupancyQuadtreeNode) -> bool: ...
    def is_node_at_threshold(self, node: OccupancyQuadtreeNode) -> bool: ...
    def get_hit_occupied_node(
        self,
        px: float,
        py: float,
        vx: float,
        vy: float,
        ignore_unknown: bool,
        max_range: float,
    ) -> Optional[OccupancyQuadtreeNode]: ...

class OccupancyQuadtree(AbstractOccupancyQuadtree):
    Setting = OccupancyQuadtreeBaseSetting

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, setting: OccupancyQuadtreeBaseSetting) -> None: ...
    @overload
    def __init__(
        self,
        map_info: GridMapInfo2D,
        image_map: npt.NDArray,
        occupied_threshold: float,
        padding: int = 0,
    ) -> None: ...
    @overload
    def __init__(self, filename: str, use_derived_constructor: bool) -> None: ...
    @property
    def tree_type(self) -> str: ...
    @property
    def setting(self) -> OccupancyQuadtreeBaseSetting: ...
    @property
    def is_node_collapsible(self) -> bool: ...
    def insert_point_cloud(
        self,
        points: npt.NDArray[np.float64],
        sensor_origin: npt.NDArray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
        discretize: bool,
    ) -> None: ...
    def insert_point_cloud_rays(
        self,
        points: npt.NDArray[np.float64],
        sensor_origin: npt.NDArray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
    ) -> None: ...
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, max_range: float, lazy_eval: bool) -> None: ...
    def sample_positions(self, num_positions: int) -> list[npt.NDArray]: ...
    @overload
    def cast_rays(
        self,
        position: npt.NDArray,
        rotation: npt.NDArray,
        angles: npt.NDArray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": npt.NDArray,
            "hit_positions": npt.NDArray,
            "hit_nodes": list[OccupancyQuadtreeNode],
            "node_depths": list[int],
        },
    ): ...
    @overload
    def cast_rays(
        self,
        positions: npt.NDArray,
        directions: npt.NDArray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": npt.NDArray,
            "hit_positions": npt.NDArray,
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
    def metric_min(self) -> npt.NDArray: ...
    @property
    def metric_max(self) -> npt.NDArray: ...
    @property
    def metric_aabb(self) -> Aabb2D: ...
    @property
    def metric_min_max(self) -> Tuple[npt.NDArray, npt.NDArray]: ...
    @property
    def metric_size(self) -> npt.NDArray: ...
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
        area_min: npt.NDArray[np.float64] = None,
        area_max: npt.NDArray[np.float64] = None,
        resolution: float = 0.1,
        padding: int = 1,
        bg_color: npt.NDArray[np.float64] = np.array([128, 128, 128, 255]),
        fg_color: npt.NDArray[np.float64] = np.array([255, 255, 255, 255]),
        occupied_color: npt.NDArray[np.float64] = np.array([0, 0, 0, 255]),
        free_color: npt.NDArray[np.float64] = np.array([255, 255, 255, 255]),
        border_color: npt.NDArray[np.float64] = np.array([0, 0, 0, 255]),
        border_thickness: int = 1,
    ) -> npt.NDArray[np.uint8]: ...

    class IteratorBase(collections.Iterable, AbstractQuadtree.QuadtreeNodeIterator):
        def __iter__(self): ...
        def __next__(self): ...
        def __eq__(self, other) -> bool: ...
        def __ne__(self, other) -> bool: ...
        @property
        def node(self) -> OccupancyQuadtreeNode: ...
        @property
        def node_aabb(self) -> Aabb2D: ...
        @property
        def key(self) -> QuadtreeKey: ...
        @property
        def index_key(self) -> QuadtreeKey: ...

    class TreeIterator(IteratorBase): ...
    class TreeInAabbIterator(IteratorBase): ...
    class LeafIterator(IteratorBase): ...
    class LeafOfNodeIterator(IteratorBase): ...
    class LeafInAabbIterator(IteratorBase): ...
    class WestLeafNeighborIterator(IteratorBase): ...
    class EastLeafNeighborIterator(IteratorBase): ...
    class NorthLeafNeighborIterator(IteratorBase): ...
    class SouthLeafNeighborIterator(IteratorBase): ...
    class NodeOnRayIterator(IteratorBase): ...

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
    def iter_node_on_ray(
        self,
        px: float,
        py: float,
        vx: float,
        vy: float,
        max_range: float = -1,
        node_padding: int = 0,
        bidirectional: bool = False,
        min_node_depth: int = 0,
        max_node_depth: int = 0,
    ) -> NodeOnRayIterator: ...

    class Drawer:
        class Setting(YamlableBase):
            area_min: npt.NDArray[np.float64]
            area_max: npt.NDArray[np.float64]
            resolution: float
            padding: int
            bg_color: npt.NDArray[np.float64]
            fg_color: npt.NDArray[np.float64]
            border_color: npt.NDArray[np.float64]
            border_thickness: int
            occupied_color: npt.NDArray[np.float64]
            free_color: npt.NDArray[np.float64]

            def __init__(self: OccupancyQuadtree.Drawer.Setting) -> None: ...

        def __init__(self, setting: OccupancyQuadtree.Drawer.Setting, quadtree: OccupancyQuadtree) -> None: ...
        @property
        def setting(self: OccupancyQuadtree.Drawer) -> OccupancyQuadtree.Drawer.Setting: ...
        @property
        def grid_map_info(self: OccupancyQuadtree.Drawer) -> GridMapInfo2D: ...
        def set_draw_tree_callback(self: OccupancyQuadtree.Drawer, callback: Callable) -> None: ...
        def set_draw_leaf_callback(self: OccupancyQuadtree.Drawer, callback: Callable) -> None: ...
        def draw_tree(self: OccupancyQuadtree.Drawer) -> npt.NDArray[np.uint8]: ...
        def draw_leaves(self: OccupancyQuadtree.Drawer) -> npt.NDArray[np.uint8]: ...

class PyObjectOccupancyQuadtreeNode(OccupancyQuadtreeNode):
    py_object: object

class PyObjectOccupancyQuadtree(OccupancyQuadtree): ...

class SurfaceMappingQuadtreeNode(OccupancyQuadtreeNode):
    class SurfaceData:
        position: npt.NDArray[np.float64]
        normal: npt.NDArray[np.float64]
        var_position: float
        var_normal: float

        def __eq__(self, other) -> bool: ...
        def __ne__(self, other) -> bool: ...

    @property
    def surface_data(self) -> SurfaceMappingQuadtreeNode.SurfaceData: ...

class SurfaceMappingQuadtree(OccupancyQuadtree): ...

class OccupancyOctreeBaseSetting(OccupancyNdTreeSetting):
    use_change_detection: bool
    use_aabb_limit: bool
    aabb: Aabb3D

    def __init__(self: OccupancyOctreeBaseSetting) -> None: ...

class OctreeKey:
    def __eq__(self, other) -> bool: ...
    def __ne__(self, other) -> bool: ...
    def __getitem__(self, item) -> int: ...
    def __hash__(self) -> int: ...

class OctreeKeyRay:
    def __len__(self) -> int: ...
    def __getitem__(self, item) -> OctreeKey: ...

class OccupancyOctreeNode(AbstractOctreeNode):
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

class AbstractOccupancyOctree(AbstractOctree):
    def write_binary(self, filename: str, prune_at_first: bool) -> bool: ...
    def read_binary(self, filename: str) -> bool: ...
    def is_node_occupied(self, node: OccupancyOctreeNode) -> bool: ...
    def is_node_at_threshold(self, node: OccupancyOctreeNode) -> bool: ...
    def get_hit_occupied_node(
        self,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        ignore_unknown: bool,
        max_range: float,
    ) -> Optional[OccupancyOctreeNode]: ...

class OccupancyOctree(AbstractOccupancyOctree):
    Setting = OccupancyOctreeBaseSetting

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, setting: OccupancyOctreeBaseSetting) -> None: ...
    @overload
    def __init__(self, filename: str, use_derived_constructor: bool) -> None: ...
    @property
    def tree_type(self) -> str: ...
    @property
    def setting(self) -> OccupancyOctreeBaseSetting: ...
    @property
    def is_node_collapsible(self) -> bool: ...
    def insert_point_cloud(
        self,
        points: npt.NDArray[np.float64],
        sensor_origin: npt.NDArray[np.float64],
        max_range: float,
        parallel: bool,
        lazy_eval: bool,
        discretize: bool,
    ) -> None: ...
    def insert_point_cloud_rays(
        self,
        points: npt.NDArray[np.float64],
        sensor_origin: npt.NDArray[np.float64],
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
    def sample_positions(self, num_positions: int) -> list[npt.NDArray]: ...
    @overload
    def cast_rays(
        self,
        position: npt.NDArray,
        rotation: npt.NDArray,
        azimuth_angles: npt.NDArray,
        elevation_angles: npt.NDArray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": list[tuple[int, int]],
            "hit_positions": list[npt.NDArray],
            "hit_nodes": list[OccupancyOctreeNode],
            "node_depths": list[int],
        },
    ): ...
    @overload
    def cast_rays(
        self,
        positions: npt.NDArray,
        directions: npt.NDArray,
        ignore_unknown: bool,
        max_range: float,
        prune_rays: bool,
        parallel: bool,
    ) -> TypedDict(
        "returns",
        {
            "hit_ray_indices": list[int],
            "hit_positions": npt.NDArray,
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
    def metric_min(self) -> npt.NDArray: ...
    @property
    def metric_max(self) -> npt.NDArray: ...
    @property
    def metric_aabb(self) -> Aabb3D: ...
    @property
    def metric_min_max(self) -> Tuple[npt.NDArray, npt.NDArray]: ...
    @property
    def metric_size(self) -> npt.NDArray: ...
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
        occupied_color: npt.NDArray[np.float64] = np.array([0.5, 0.5, 0.5]),
        border_color: npt.NDArray[np.float64] = np.array([0.0, 0.0, 0.0]),
        window_width: int = 1920,
        window_height: int = 1080,
        window_left: int = 50,
        window_top: int = 50,
    ) -> npt.NDArray[np.uint8]: ...

    class IteratorBase(collections.Iterable, AbstractOctree.OctreeNodeIterator):
        def __iter__(self): ...
        def __next__(self): ...
        def __eq__(self, other) -> bool: ...
        def __ne__(self, other) -> bool: ...
        @property
        def node(self) -> OccupancyOctreeNode: ...
        @property
        def node_aabb(self) -> Aabb3D: ...
        @property
        def key(self) -> OctreeKey: ...
        @property
        def index_key(self) -> OctreeKey: ...

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
    class NodeOnRayIterator(IteratorBase): ...

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
    def iter_node_on_ray(
        self,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        max_range: float = -1,
        node_padding: float = 0,
        bidirectional: bool = False,
        min_node_depth: int = 0,
        max_node_depth: int = 0,
    ) -> NodeOnRayIterator: ...

    class Drawer:
        class Setting(YamlableBase):
            area_min: npt.NDArray[np.float64]
            area_max: npt.NDArray[np.float64]
            occupied_only: bool
            occupied_color: npt.NDArray[np.float64]
            draw_node_boxes: bool
            draw_node_borders: bool

            def __init__(self: OccupancyOctree.Drawer.Setting) -> None: ...

        def __init__(self, setting: OccupancyOctree.Drawer.Setting, octree: OccupancyOctree) -> None: ...
        @property
        def setting(self: OccupancyOctree.Drawer) -> OccupancyOctree.Drawer.Setting: ...
        @property
        def grid_map_info(self: OccupancyOctree.Drawer) -> GridMapInfo2D: ...
        def set_draw_tree_callback(self: OccupancyOctree.Drawer, callback: Callable) -> None: ...
        def set_draw_leaf_callback(self: OccupancyOctree.Drawer, callback: Callable) -> None: ...
        def draw_tree(self: OccupancyOctree.Drawer) -> npt.NDArray[np.uint8]: ...
        def draw_leaves(self: OccupancyOctree.Drawer) -> npt.NDArray[np.uint8]: ...

class PyObjectOccupancyOctreeNode(OccupancyOctreeNode):
    py_object: object

class PyObjectOccupancyOctree(OccupancyOctree): ...

class SurfaceMappingOctreeNode(OccupancyOctreeNode):
    class SurfaceData:
        position: npt.NDArray[np.float64]
        normal: npt.NDArray[np.float64]
        var_position: float
        var_normal: float

        def __eq__(self, other) -> bool: ...
        def __ne__(self, other) -> bool: ...

    @property
    def surface_data(self) -> SurfaceMappingOctreeNode.SurfaceData: ...

class SurfaceMappingOctree(OccupancyOctree): ...

class Surface2D:
    @overload
    def __init__(
        self: Surface2D,
        vertices: npt.NDArray[np.float64],
        normals: npt.NDArray[np.float64],
        lines2vertices: npt.NDArray[np.float64],
        objects2lines: npt.NDArray[np.float64],
        outside_flags: npt.NDArray[np.bool_],
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
    def vertices(self: Surface2D) -> npt.NDArray[np.float64]: ...
    @property
    def normals(self: Surface2D) -> npt.NDArray[np.float64]: ...
    @property
    def lines_to_vertices(self: Surface2D) -> npt.NDArray[np.float64]: ...
    @property
    def objects_to_lines(self: Surface2D) -> npt.NDArray[np.float64]: ...
    @property
    def vertices_to_objects(self: Surface2D) -> npt.NDArray[np.float64]: ...
    @property
    def outside_flags(self: Surface2D) -> npt.NDArray[np.bool_]: ...
    @property
    def normals_available(self: Surface2D) -> bool: ...
    @property
    def outside_flags_available(self: Surface2D) -> bool: ...
    def get_object_vertices(self: Surface2D, index_object: int) -> npt.NDArray[np.float64]: ...
    def get_object_normals(self: Surface2D, index_object: int) -> npt.NDArray[np.float64]: ...
    def get_vertex_neighbors(self: Surface2D, index_vertex: int) -> npt.NDArray[np.float64]: ...

class Space2D:
    class SignMethod(IntEnum):
        kPointNormal = 0
        kLineNormal = 1
        kPolygon = 2

    @overload
    def __init__(
        self: Space2D,
        ordered_object_vertices: list[npt.NDArray[np.float64]],
        ordered_object_normals: list[npt.NDArray[np.float64]],
    ): ...
    @overload
    def __init__(
        self: Space2D,
        ordered_object_vertices: list[npt.NDArray[np.float64]],
        outside_flags: npt.NDArray[np.bool_],
        delta: float = 0.01,
        parallel: bool = False,
    ): ...
    @overload
    def __init__(
        self: Space2D,
        map_image: npt.NDArray[np.float64],
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
    def generate_map_image(self: Space2D, map_image_info: GridMapInfo2D) -> npt.NDArray[np.uint8]: ...
    @overload
    def compute_sdf_image(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        sign_method: SignMethod = SignMethod.kLineNormal,
        use_kdtree: bool = False,
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    @overload
    def compute_sdf(
        self: Space2D,
        query_points: npt.NDArray[np.float64],
        sign_method: SignMethod = SignMethod.kLineNormal,
        use_kdtree: bool = False,
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    def compute_sdf_with_kdtree(self: Space2D, q: npt.NDArray[np.float64], sign_method: SignMethod) -> float: ...
    def compute_sdf_greedily(self: Space2D, q: npt.NDArray[np.float64], sign_method: SignMethod) -> float: ...
    @overload
    def compute_ddf(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        query_directions: npt.NDArray[np.float64],
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    @overload
    def compute_ddf(
        self: Space2D,
        query_points: npt.NDArray[np.float64],
        query_directions: npt.NDArray[np.float64],
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    @overload
    def compute_sddf_v1(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        query_directions: npt.NDArray[np.float64],
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    @overload
    def compute_sddf_v1(
        self: Space2D,
        query_points: npt.NDArray[np.float64],
        query_directions: npt.NDArray[np.float64],
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    @overload
    def compute_sddf_v2(
        self: Space2D,
        grid_map_info: GridMapInfo2D,
        query_directions: npt.NDArray[np.float64],
        sign_method: SignMethod = SignMethod.kLineNormal,
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...
    @overload
    def compute_sddf_v2(
        self: Space2D,
        query_points: npt.NDArray[np.float64],
        query_directions: npt.NDArray[np.float64],
        sign_method: SignMethod = SignMethod.kLineNormal,
        parallel: bool = False,
    ) -> npt.NDArray[np.float64]: ...

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
    def angles(self: Lidar2D) -> npt.NDArray: ...
    @property
    def ray_directions_in_frame(self: Lidar2D) -> npt.NDArray: ...
    @overload
    def scan(self: Lidar2D, rotation_angle: float, translation: npt.NDArray, parallel: bool = False) -> npt.NDArray: ...
    @overload
    def scan(self: Lidar2D, rotation: npt.NDArray, translation: npt.NDArray, parallel: bool = False) -> npt.NDArray: ...
    def scan_multi_poses(self: Lidar2D, poses: list[npt.NDArray], parallel: bool = False) -> npt.NDArray: ...

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
        rotation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        angles: npt.NDArray[np.float64],
        ranges: npt.NDArray[np.float64],
        partition_rays: bool = False,
    ): ...
    @property
    def setting(self) -> LidarFrame2D.Setting: ...
    @property
    def num_rays(self) -> int: ...
    @property
    def num_hit_rays(self) -> int: ...
    @property
    def rotation_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def rotation_angle(self) -> float: ...
    @property
    def translation_vector(self) -> npt.NDArray[np.float64]: ...
    @property
    def pose_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def angles_in_frame(self) -> npt.NDArray[np.float64]: ...
    @property
    def angles_in_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def ranges(self) -> npt.NDArray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> npt.NDArray[np.float64]: ...
    @property
    def ray_directions_in_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def end_points_in_frame(self) -> npt.NDArray[np.float64]: ...
    @property
    def end_points_in_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def hit_ray_indices(self) -> npt.NDArray[np.int64]: ...
    @property
    def hit_points_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def max_valid_range(self) -> float: ...
    @property
    def partitions(self) -> list[LidarFramePartition2D]: ...
    @property
    def is_valid(self) -> bool: ...
    def compute_closest_end_point(
        self, position: npt.NDArray[np.float64]
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
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    def compute_rays_at(
        self, position_world: npt.NDArray[np.float64]
    ) -> TypedDict(
        "returns",
        {
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
        shape_metric_vertices: npt.NDArray[np.float64],
    ) -> None: ...
    def update(
        self: LogOddMap2D,
        position: npt.NDArray[np.float64],
        theta: float,
        angles_body: npt.NDArray[np.float64],
        ranges: npt.NDArray[np.float64],
    ) -> None: ...
    def compute_statistics_of_lidar_frame(
        self: LogOddMap2D,
        position: npt.NDArray[np.float64],
        theta: float,
        angles_body: npt.NDArray[np.float64],
        ranges: npt.NDArray[np.float64],
        clip_ranges: bool,
    ) -> Tuple[int, int, int, int]: ...
    @property
    def setting(self: LogOddMap2D) -> Setting: ...
    @property
    def log_map(self: LogOddMap2D) -> npt.NDArray[np.float64]: ...
    @property
    def possibility_map(self: LogOddMap2D) -> npt.NDArray[np.float64]: ...
    @property
    def occupancy_map(self: LogOddMap2D) -> npt.NDArray[np.uint8]: ...
    @property
    def unexplored_mask(self: LogOddMap2D) -> npt.NDArray[np.uint8]: ...
    @property
    def occupied_mask(self: LogOddMap2D) -> npt.NDArray[np.unit8]: ...
    @property
    def free_mask(self: LogOddMap2D) -> npt.NDArray[np.uint8]: ...
    @property
    def num_unexplored_cells(self: LogOddMap2D) -> int: ...
    @property
    def num_occupied_cells(self: LogOddMap2D) -> int: ...
    @property
    def num_free_cells(self: LogOddMap2D) -> int: ...
    def get_frontiers(
        self: LogOddMap2D, clean_at_first: bool = True, approx_iters: int = 4
    ) -> list[npt.NDArray[np.int32]]: ...

class Lidar3D:
    class Setting(YamlableBase):
        azimuth_min: float
        azimuth_max: float
        elevation_min: float
        elevation_max: float
        num_azimuth_lines: int
        num_elevation_lines: int

    @overload
    def __init__(self, setting: Setting, vertices: npt.NDArray, triangles: npt.NDArray) -> None: ...
    @overload
    def __init__(self, setting: Setting, vertices: list[npt.NDArray], triangles: list[npt.NDArray]) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def azimuth_angles(self) -> npt.NDArray[np.float64]: ...
    @property
    def elevation_angles(self) -> npt.NDArray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> npt.NDArray[np.float64]: ...
    def scan(
        self,
        orientation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        add_noise: bool = False,
        noise_stddev: float = 0.03,
    ) -> npt.NDArray[np.float64]: ...

class DepthCamera3D:
    class Setting(YamlableBase):
        image_height: int
        image_width: int
        camera_fx: float
        camera_fy: float
        camera_cx: float
        camera_cy: float

    @overload
    def __init__(self, setting: Setting, vertices: npt.NDArray, triangles: npt.NDArray) -> None: ...
    @overload
    def __init__(self, setting: Setting, vertices: list[npt.NDArray], triangles: list[npt.NDArray]) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def ray_directions_in_frame(self) -> npt.NDArray[np.float64]: ...
    def scan(
        self,
        rotation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        add_noise: bool = False,
        noise_stddev: float = 0.03,
    ) -> npt.NDArray[np.float64]: ...

class LidarFramePartition3D: ...

class RangeSensorFrame3D:
    class Setting(YamlableBase):
        row_margin: int
        col_margin: int
        valid_range_min: float
        valid_range_max: float
        discontinuity_factor: float
        rolling_diff_discount: float
        min_partition_size: int

    @property
    def num_rays(self) -> int: ...
    @property
    def num_hit_rays(self) -> int: ...
    @property
    def rotation_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def translation_vector(self) -> npt.NDArray[np.float64]: ...
    @property
    def pose_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def ranges(self) -> npt.NDArray[np.float64]: ...
    @property
    def frame_coords(self) -> npt.NDArray[np.float64]: ...
    @property
    def ranges(self) -> npt.NDArray[np.float64]: ...
    @property
    def ray_directions_in_frame(self) -> npt.NDArray[np.float64]: ...
    @property
    def ray_directions_in_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def end_points_in_frame(self) -> npt.NDArray[np.float64]: ...
    @property
    def end_points_in_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def hit_ray_indices(self) -> npt.NDArray[np.int64]: ...
    @property
    def hit_points_world(self) -> npt.NDArray[np.float64]: ...
    @property
    def max_valid_range(self) -> float: ...
    @property
    def hit_mask(self) -> npt.NDArray[np.bool_]: ...
    @property
    def is_valid(self) -> bool: ...
    def points_is_in_frame(self, xyz_frame: npt.NDArray[np.float64]) -> npt.NDArray[np.bool_]: ...
    def coords_is_in_frame(self, coords_frame: npt.NDArray[np.float64]) -> npt.NDArray[np.bool_]: ...
    def compute_frame_coords(self, xyz_frame: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: ...
    def world_to_frame_so3(self, dir_world: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: ...
    def frame_to_world_so3(self, dir_frame: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: ...
    def world_to_frame_se3(self, xyz_world: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: ...
    def frame_to_world_se3(self, xyz_frame: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: ...
    def compute_closest_end_point(
        self, position_world: npt.NDArray[np.float64], brute_force: bool
    ) -> TypedDict("returns", {"row_index": int, "col_index": int, "distance": float,},): ...
    @overload
    def sample_along_rays(
        self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    @overload
    def sample_along_rays(
        self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    def sample_near_surface(
        self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float
    ) -> TypedDict(
        "returns",
        {
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
            "positions_world": npt.NDArray[np.float64],
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    def compute_rays_at(
        self, position_world: npt.NDArray[np.float64]
    ) -> TypedDict(
        "returns",
        {
            "directions_world": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
            "visible_hit_point_indices": list[int],
        },
    ): ...

class LidarFrame3D:
    class Setting(RangeSensorFrame3D.Setting):
        azimuth_min: float
        azimuth_max: float
        elevation_min: float
        elevation_max: float
        num_azimuth_lines: int
        num_elevation_lines: int

    def __init__(self: LidarFrame3D, setting: Setting) -> None: ...
    def reset(self) -> None: ...
    def update_ranges(
        self,
        rotation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        ranges: npt.NDArray[np.float64],
        partition_rays: bool = False,
    ) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def num_azimuth_lines(self) -> int: ...
    @property
    def num_elevation_lines(self) -> int: ...
    @property
    def is_partitioned(self) -> bool: ...
    @property
    def partitions(self) -> list[LidarFramePartition3D]: ...

class DepthFrame3D:
    class Setting(RangeSensorFrame3D.Setting):
        camera_to_optical: npt.NDArray[np.float64]
        image_height: int
        image_width: int
        camera_fx: float
        camera_fy: float
        camera_cx: float
        camera_cy: float

        def resize(self, factor: float) -> Tuple[int, int]: ...

    def __init__(self: DepthFrame3D.Setting, setting: Setting) -> None: ...
    def reset(self) -> None: ...
    @overload
    def update_ranges(
        self,
        orientation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        depth: npt.NDArray[np.float64],
        depth_scaled: bool = True,
        partition_rays: bool = False,
    ) -> None: ...
    @overload
    def update_ranges(
        self,
        orientation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        depth_file: str,
        partition_rays: bool = False,
    ) -> None: ...
    @property
    def setting(self) -> Setting: ...
    @property
    def image_height(self) -> int: ...
    @property
    def image_width(self) -> int: ...
    @property
    def camera_extrinsic_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def camera_intrinsic_matrix(self) -> npt.NDArray[np.float64]: ...
    @staticmethod
    def depth_image_to_depth(depth_image: npt.NDArray[np.float64], depth_scale: float) -> npt.NDArray[np.float64]: ...
    @staticmethod
    def depth_to_depth_image(depth: npt.NDArray[np.float64], depth_scale: float) -> npt.NDArray[np.float64]: ...
    @property
    def is_partitioned(self) -> bool: ...
    @property
    def partitions(self) -> list[LidarFramePartition3D]: ...

class AbstractSurfaceMapping:
    class Setting(YamlableBase):
        @staticmethod
        def create(mapping_type: str) -> AbstractSurfaceMapping.Setting: ...

    @staticmethod
    def create_surface_mapping(
        mapping_type: str, setting: AbstractSurfaceMapping.Setting
    ) -> AbstractSurfaceMapping: ...
    def write(self, filename: str) -> bool: ...
    def read(self, filename: str) -> bool: ...

class AbstractSurfaceMapping2D(AbstractSurfaceMapping):
    @property
    def quadtree(self) -> SurfaceMappingQuadtree: ...
    @property
    def sensor_noise(self) -> float: ...
    @property
    def cluster_level(self) -> int: ...
    def update(
        self, rotation: npt.NDArray[np.float64], translation: npt.NDArray[np.float64], ranges: npt.NDArray[np.float64]
    ) -> None: ...

class AbstractSurfaceMapping3D(AbstractSurfaceMapping):
    @property
    def octree(self) -> SurfaceMappingOctree: ...
    @property
    def sensor_noise(self) -> float: ...
    @property
    def cluster_level(self) -> int: ...
    def update(
        self, rotation: npt.NDArray[np.float64], translation: npt.NDArray[np.float64], ranges: npt.NDArray[np.float64]
    ) -> None: ...

class Primitive2D:
    class Type(IntEnum):
        kLine2D = 0
        kSegment2D = 1
        kRay2D = 2
        kAxisAlignedRectangle = 3
        kRectangle = 4
        kEllipse = 5

    id: int

    @property
    def type(self) -> Type: ...
    def is_inside(self, point: npt.NDArray[np.float64]) -> bool: ...
    def is_on_boundary(self, point: npt.NDArray[np.float64]) -> bool: ...
    @overload
    def compute_intersections(self, line) -> list[npt.NDArray[np.float64]]: ...
    @overload
    def compute_intersections(self, segment) -> list[npt.NDArray[np.float64]]: ...
    @overload
    def compute_intersections(self, ray) -> list[npt.NDArray[np.float64]]: ...
    @property
    def orientation_angle(self) -> float: ...

class Line2D(Primitive2D):
    p0: npt.NDArray[np.float64]
    p1: npt.NDArray[np.float64]

    def __init__(self, id: int, p0: npt.NDArray[np.float64], p1: npt.NDArray[np.float64]) -> None: ...

class Segment2D(Line2D):
    def __init__(self, id: int, p0: npt.NDArray[np.float64], p1: npt.NDArray[np.float64]) -> None: ...

class Ray2D(Primitive2D):
    origin: npt.NDArray[np.float64]
    direction: npt.NDArray[np.float64]

    def __init__(self, id: int, origin: npt.NDArray[np.float64], direction: npt.NDArray[np.float64]) -> None: ...

class AxisAlignedRectangle2D(Primitive2D, Aabb2D):
    def __init__(self, id: int, center: npt.NDArray[np.float64], half_sizes: npt.NDArray[np.float64]) -> None: ...

class Rectangle2D(Primitive2D):
    def __init__(
        self, id: int, center: npt.NDArray[np.float64], half_sizes: npt.NDArray[np.float64], angle: float
    ) -> None: ...

    center: npt.NDArray[np.float64]

    @property
    def half_sizes(self) -> npt.NDArray[np.float64]: ...
    @property
    def rotation_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def translate(self, translation: npt.NDArray[np.float64]) -> None: ...

    orientation_angle: float

    def compute_points_on_boundary(self, num_points: int) -> npt.NDArray[np.float64]: ...

class Ellipse2D(Primitive2D):
    def __init__(self, id: int, center: npt.NDArray[np.float64], a: float, b: float, angle: float) -> None: ...

    center: npt.NDArray[np.float64]

    @property
    def radii(self) -> npt.NDArray[np.float64]: ...
    @property
    def rotation_matrix(self) -> npt.NDArray[np.float64]: ...
    @property
    def translate(self, translation: npt.NDArray[np.float64]) -> None: ...
    @property
    def rotate(self, angle: float) -> None: ...

    orientation_angle: float

    def compute_points_on_boundary(
        self, num_points: int, start_angle: float, end_angle: float
    ) -> npt.NDArray[np.float64]: ...

class Primitive3D:
    class Type(IntEnum):
        kLine3D = 0
        kSegment3D = 1
        kRay3D = 2
        kPlane = 3
        kTriangle = 4
        kAxisAlignedBox = 5
        kBox = 6
        kEllipsoid = 7

    id: int

    @property
    def type(self) -> Type: ...
    def is_inside(self, point: npt.NDArray[np.float64]) -> bool: ...

class Box(Primitive3D):
    def __init__(
        self,
        id: int,
        center: npt.NDArray[np.float64],
        half_sizes: npt.NDArray[np.float64],
        rotation: npt.NDArray[np.float64],
    ) -> None: ...

    center: npt.NDArray[np.float64]

    @property
    def half_sizes(self) -> npt.NDArray[np.float64]: ...

    rotation_matrix: npt.NDArray[np.float64]

    def translate(self, translation: npt.NDArray[np.float64]) -> None: ...
    def rotate(self, rotation: npt.NDArray[np.float64]) -> None: ...

class Ellipsoid(Primitive3D):
    def __init__(
        self,
        id: int,
        center: npt.NDArray[np.float64],
        radius: npt.NDArray[np.float64],
        rotation: npt.NDArray[np.float64],
    ) -> None: ...

    center: npt.NDArray[np.float64]

    @property
    def radii(self) -> npt.NDArray[np.float64]: ...

    rotation_matrix: npt.NDArray[np.float64]

    def translate(self, translation: npt.NDArray[np.float64]) -> None: ...
    def rotate(self, rotation: npt.NDArray[np.float64]) -> None: ...

def create_ellipsoid_mesh(
    a: float, b: float, c: float, num_azimuths: int = 360, num_elevations: int = 180
) -> tuple[list[npt.NDArray], list[npt.NDArray]]: ...

class Trajectory:
    @staticmethod
    def load_2d(filename: str, binary: bool) -> list[npt.NDArray[np.float64]]: ...
    @staticmethod
    def load_3d(filename: str, binary: bool) -> list[npt.NDArray[np.float64]]: ...
    @staticmethod
    def load_se2(filename: str, binary: bool) -> list[tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]]: ...
    @staticmethod
    def load_se3(filename: str, binary: bool) -> list[tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]]: ...
