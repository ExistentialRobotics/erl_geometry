from enum import IntEnum
from typing import Callable
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple
from typing import TypedDict
from typing import overload

import numpy as np
import numpy.typing as npt

from erl_common.storage import GridMapInfo2D
from erl_common.storage import GridMapInfo3D
from erl_common.storage import GridMapUnsigned2D
from erl_common.storage import GridMapUnsigned3D
from erl_common.yaml import YamlableBase

__all__ = [
    "marching_square",
    "bresenham_2d",
    "compute_pixel_of_polygon_contour",
    "winding_number",
    "compute_nearest_distance_from_point_to_line_segment_2d",
    "compute_intersection_between_ray_and_segment_2d",
    "compute_intersection_between_ray_and_aabb_2d",
    "Aabb2D",
    "Aabb3D",
    "Node",
    "NodeData",
    "NodeContainer",
    "NodeContainerMultiTypes",
    "IncrementalQuadtree",
    "Surface2D",
    "Space2D",
    "Lidar2D",
    "LidarFrame2D",
    "LogOddMap2D",
    "CollisionCheckerBase",
    "PointCollisionChecker2D",
    "PointCollisionChecker3D",
    "GridCollisionCheckerSe2",
    "GridCollisionChecker3D",
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
def compute_intersection_between_ray_and_segment_2d(
    ray_start_point: npt.NDArray[np.float64],
    ray_direction: npt.NDArray[np.float64],
    segment_point1: npt.NDArray[np.float64],
    segment_point2: npt.NDArray[np.float64],
) -> Tuple[float, float]:
    """
    Returns:
        Tuple[float, float]: (t, distance) where intersection = t * segment_point1 + (1 - t) * segment_point2
    """
    ...

def compute_intersection_between_ray_and_aabb_2d(
    ray_start_point: npt.NDArray[np.float64],
    ray_direction: npt.NDArray[np.float64],
    aabb_min: npt.NDArray[np.float64],
    aabb_max: npt.NDArray[np.float64],
) -> Tuple[float, bool]: ...

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
    @overload
    def __contains__(self, point: npt.NDArray) -> bool: ...
    @overload
    def __contains__(self: Aabb2D, aabb: Aabb2D) -> bool: ...
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
    def __init__(self: Aabb2D, center: npt.NDArray, half_size: float): ...
    @property
    def center(self: Aabb2D) -> npt.NDArray: ...
    @property
    def half_sizes(self: Aabb2D) -> float: ...
    @overload
    def __contains__(self, point: npt.NDArray) -> bool: ...
    @overload
    def __contains__(self: Aabb2D, aabb: Aabb2D) -> bool: ...
    def corner(self: Aabb2D, corner_type: int) -> npt.NDArray: ...
    def intersects(self: Aabb2D, aabb: Aabb2D) -> bool: ...

class Node:
    def __init__(self: Node, type: int, position: npt.NDArray[np.float64], node_data: NodeData = None): ...
    @property
    def position(self: Node) -> npt.NDArray[np.float64]: ...
    @property
    def type(self: Node) -> int: ...
    @property
    def node_data(self: Node) -> NodeData: ...

class NodeData:
    def __str__(self) -> str: ...

class NodeContainer:
    @property
    def node_types(self: NodeContainer) -> List[int]: ...
    def node_type_name(self: NodeContainer, type: int) -> str: ...
    def size(self: NodeContainer) -> int: ...
    def size_of_type(self: NodeContainer, type: int) -> int: ...
    def empty(self: NodeContainer) -> bool: ...
    def empty_of_type(self: NodeContainer, type: int) -> bool: ...
    def clear(self: NodeContainer) -> None: ...
    def collect_nodes(self: NodeContainer) -> List[Node]: ...
    def collect_nodes_of_type(self: NodeContainer, type: int) -> List[Node]: ...
    def collect_nodes_of_type_in_aabb_2d(self: NodeContainer, type: int, aabb_2d: Aabb2D) -> List[Node]: ...
    def collect_nodes_of_type_in_aabb_3d(self: NodeContainer, type: int, aabb_3d: Aabb3D) -> List[Node]: ...
    def insert(self: NodeContainer, node: Node) -> bool: ...
    def remove(self: NodeContainer, node: Node) -> bool: ...

class NodeContainerMultiTypes(NodeContainer):
    class Setting(YamlableBase):
        num_node_types: int
        node_type_capacity: List[int]
        node_type_min_squared_distance: List[float]

        @overload
        def __init__(self: NodeContainerMultiTypes.Setting) -> None: ...
        @overload
        def __init__(
            self: NodeContainerMultiTypes.Setting,
            num_node_types: int,
            capacity_per_type: int = 1,
            min_squared_distance: float = 0.04,
        ) -> None: ...

    def __init__(self: NodeContainerMultiTypes, setting: NodeContainerMultiTypes.Setting) -> None: ...

class IncrementalQuadtree:
    class Children:
        class Type(IntEnum):
            kNorthWest = 0
            kNorthEast = 1
            kSouthWest = 2
            kSouthEast = 3
            kRoot = 4
        def __getitem__(self: IncrementalQuadtree.Children, item: int) -> IncrementalQuadtree: ...
        def reset(self: IncrementalQuadtree.Children) -> None: ...
        def north_west(self: IncrementalQuadtree.Children) -> IncrementalQuadtree: ...
        def north_east(self): ...
        def south_west(self): ...
        def south_east(self): ...

    class Setting(YamlableBase):
        nodes_in_leaves_only: bool
        cluster_half_area_size: float
        max_half_area_size: float
        min_half_area_size: float

        def __init__(self: IncrementalQuadtree.Setting): ...

    @overload
    def __init__(
        self: IncrementalQuadtree,
        setting: IncrementalQuadtree.Setting,
        area: Aabb2D,
        node_container_constructor: Callable[[], NodeContainer],
    ): ...
    @overload
    def __init__(
        self: IncrementalQuadtree,
        setting: IncrementalQuadtree,
        area: Aabb2D,
        node_container_constructor: Callable[[], NodeContainer],
        parent: IncrementalQuadtree,
        child_type: IncrementalQuadtree.Children.Type,
    ): ...
    @property
    def setting(self: IncrementalQuadtree.Setting) -> Setting: ...
    @property
    def root(self: IncrementalQuadtree) -> IncrementalQuadtree: ...
    @property
    def area(self: IncrementalQuadtree) -> Aabb2D: ...
    @property
    def child_type(self: IncrementalQuadtree) -> Children.Type: ...
    @property
    def parent(self: IncrementalQuadtree) -> IncrementalQuadtree: ...
    @property
    def children(self: IncrementalQuadtree) -> Children: ...
    @property
    def is_root(self: IncrementalQuadtree) -> bool: ...
    @property
    def is_leaf(self: IncrementalQuadtree) -> bool: ...
    @property
    def is_empty(self: IncrementalQuadtree) -> bool: ...
    @property
    def is_empty_of_type(self, type: int): ...
    @property
    def is_in_cluster(self: IncrementalQuadtree) -> bool: ...
    @property
    def is_expandable(self: IncrementalQuadtree) -> bool: ...
    @property
    def is_subdividable(self: IncrementalQuadtree) -> bool: ...
    def insert(
        self: IncrementalQuadtree, node: Node
    ) -> Tuple[Optional[IncrementalQuadtree], Optional[IncrementalQuadtree]]: ...
    def remove(self: IncrementalQuadtree, node: Node) -> bool: ...
    def collect_trees(
        self: IncrementalQuadtree, qualify: Callable[[IncrementalQuadtree], bool]
    ) -> List[IncrementalQuadtree]: ...
    def collect_non_empty_clusters(self: IncrementalQuadtree, area: Aabb2D) -> List[IncrementalQuadtree]: ...
    def collect_nodes(self: IncrementalQuadtree) -> List[Node]: ...
    def collect_nodes_of_type(self: IncrementalQuadtree, type: int) -> List[Node]: ...
    def collect_nodes_of_type_in_area(self: IncrementalQuadtree, type: int, area: Aabb2D) -> List[Node]: ...
    @property
    def node_types(self: IncrementalQuadtree): ...
    @overload
    def ray_tracing(
        self: IncrementalQuadtree,
        ray_origin: npt.NDArray[np.float64],
        ray_direction: npt.NDArray[np.float64],
        hit_distance_threshold: float,
    ) -> Tuple[float, Node]: ...
    @overload
    def ray_tracing(
        self: IncrementalQuadtree,
        ray_origins: npt.NDArray[np.float64],
        ray_directions: npt.NDArray[np.float64],
        hit_distance_threshold: float,
    ) -> Tuple[List[float], List[Node]]: ...
    @overload
    def ray_tracing(
        self: IncrementalQuadtree,
        node_type: int,
        ray_origin: npt.NDArray[np.float64],
        ray_direction: npt.NDArray[np.float64],
        hit_distance_threshold: float,
    ) -> Tuple[float, Node]: ...
    @overload
    def ray_tracing(
        self: IncrementalQuadtree,
        node_type: int,
        ray_origins: npt.NDArray[np.float64],
        ray_directions: npt.NDArray[np.float64],
        hit_distance_threshold: float,
    ) -> Tuple[List[float], List[Node]]: ...
    def plot(
        self: IncrementalQuadtree,
        grid_map_info: GridMapInfo2D,
        node_types: List[int],
        node_type_colors: Dict[int, npt.NDArray[np.int32]],
        node_type_radius: Dict[int, int],
        bg_color: npt.NDArray[np.float64] = np.array([255, 255, 255]),
        area_rect_color: npt.NDArray[np.float64] = np.array([0, 0, 0]),
        area_rect_thickness: int = 2,
        tree_data_color: npt.NDArray[np.float64] = np.array([255, 0, 0]),
        tree_data_radius: int = 2,
    ) -> npt.NDArray[np.uint8]: ...
    def __str__(self: IncrementalQuadtree) -> str: ...

class QuadtreeKey:
    def __eq__(self, other) -> bool: ...
    def __ne__(self, other) -> bool: ...
    def __getitem__(self, item) -> int: ...

class QuadtreeKeyRay:
    def __len__(self) -> int: ...
    def __getitem__(self, item) -> QuadtreeKey: ...

class OccupancyQuadtreeNode:
    @property
    def occupancy(self) -> float: ...
    @property
    def log_odds(self) -> float: ...
    @property
    def mean_child_log_odds(self) -> float: ...
    @property
    def max_child_log_odds(self) -> float: ...
    def add_log_odds(self, log_odds: float) -> None: ...

class OccupancyQuadtree:

    class Setting(YamlableBase):
        log_odd_min: float
        log_odd_max: float
        probability_hit: float
        probability_miss: float
        probability_occupied: float
        resolution: float
        use_change_detection: bool
        use_aabb_limit: bool
        aabb: Aabb2D

    @overload
    def __init__(self, resolution: float) -> None: ...
    @overload
    def __init__(self, setting: Setting) -> None: ...

    @property
    def tree_type(self) -> str: ...
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
        lazy_eval: bool,
    ) -> None: ...
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, max_range: float, lazy_eval: bool) -> None: ...
    def cast_ray(
        self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float
    ) -> Tuple[bool, float, float]: ...
    @overload
    def update_node(self, x: float, y: float, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    @overload
    def update_node(self, key: QuadtreeKey, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    @overload
    def update_node(self, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    @overload
    def update_node(self, key: QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode: ...
    def update_inner_occupancy(self) -> None: ...
    def to_max_likelihood(self) -> None: ...
    @property
    def number_of_nodes(self) -> int: ...
    resolution: float
    @property
    def max_tree_depth(self) -> int: ...
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
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> Optional[List[Tuple[float, float]]]: ...
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
    def search(self, x: float, y: float) -> Tuple[Optional[OccupancyQuadtreeNode], int]: ...
    @overload
    def search(self, key: QuadtreeKey) -> Tuple[Optional[OccupancyQuadtreeNode], int]: ...
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> Optional[OccupancyQuadtreeNode]: ...
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> OccupancyQuadtreeNode: ...

    class IteratorBase:
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
        def depth(self) -> int: ...
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
    class LeafNeighborOnWestIterator(IteratorBase): ...
    class LeafNeighborOnEastIterator(IteratorBase): ...
    class LeafNeighborOnNorthIterator(IteratorBase): ...
    class LeafNeighborOnSouthIterator(IteratorBase): ...
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
    def iter_leaf_neighbor_on_west(self, x: float, y: float, max_leaf_depth: int = 0) -> LeafNeighborOnWestIterator: ...
    @overload
    def iter_leaf_neighbor_on_west(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> LeafNeighborOnWestIterator: ...
    @overload
    def iter_leaf_neighbor_on_east(self, x: float, y: float, max_leaf_depth: int = 0) -> LeafNeighborOnEastIterator: ...
    @overload
    def iter_leaf_neighbor_on_east(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> LeafNeighborOnEastIterator: ...
    @overload
    def iter_leaf_neighbor_on_north(
        self, x: float, y: float, max_leaf_depth: int = 0
    ) -> LeafNeighborOnNorthIterator: ...
    @overload
    def iter_leaf_neighbor_on_north(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> LeafNeighborOnNorthIterator: ...
    @overload
    def iter_leaf_neighbor_on_south(
        self, x: float, y: float, max_leaf_depth: int = 0
    ) -> LeafNeighborOnSouthIterator: ...
    @overload
    def iter_leaf_neighbor_on_south(
        self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = 0
    ) -> LeafNeighborOnSouthIterator: ...
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
        ordered_object_vertices: List[npt.NDArray[np.float64]],
        ordered_object_normals: List[npt.NDArray[np.float64]],
    ): ...
    @overload
    def __init__(
        self: Space2D,
        ordered_object_vertices: List[npt.NDArray[np.float64]],
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
    def __init__(self: Lidar2D, space2d: Space2D): ...

    pose: npt.NDArray[np.float64]
    translation: npt.NDArray[np.float64]

    @property
    def rotation(self: Lidar2D) -> npt.NDArray[np.float64]: ...
    @overload
    def set_rotation(self: Lidar2D, rotation_matrix: npt.NDArray) -> None: ...
    @overload
    def set_rotation(self: Lidar2D, angle: float) -> None: ...

    min_angle: float
    max_angle: float
    num_lines: int

    @property
    def angles(self: Lidar2D) -> npt.NDArray: ...
    @property
    def ray_directions(self: Lidar2D) -> npt.NDArray: ...
    @property
    def oriented_ray_directions(self: Lidar2D) -> npt.NDArray: ...

    mode: Mode
    sign_method: Space2D.SignMethod

    def scan(self: Lidar2D, parallel: bool = False) -> npt.NDArray: ...
    @overload
    def scan_multi_poses(self: Lidar2D, poses: List[npt.NDArray], parallel: bool = False) -> npt.NDArray: ...
    @overload
    def scan_multi_poses(
        self: Lidar2D,
        xs: npt.NDArray,
        ys: npt.NDArray,
        thetas: npt.NDArray,
        parallel: bool = False,
    ) -> npt.NDArray: ...
    def get_rays(self: Lidar2D, parallel: bool = False) -> npt.NDArray: ...
    @overload
    def get_rays_of_multi_poses(self: Lidar2D, poses: List[npt.NDArray], parallel: bool = False) -> npt.NDArray: ...
    @overload
    def get_rays_of_multi_poses(
        self: Lidar2D,
        xs: npt.NDArray,
        ys: npt.NDArray,
        thetas: npt.NDArray,
        parallel: bool = False,
    ) -> npt.NDArray: ...

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
        min_partition_size: int
    def __init__(self: LidarFrame2D, setting: LidarFrame2D.Setting) -> None: ...
    def update(
        self,
        rotation: npt.NDArray[np.float64],
        translation: npt.NDArray[np.float64],
        angles: npt.NDArray[np.float64],
        ranges: npt.NDArray[np.float64],
    ): ...
    @property
    def setting(self) -> LidarFrame2D.Setting: ...
    @property
    def num_rays(self) -> int: ...
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
    def max_valid_range(self) -> float: ...
    @property
    def partitions(self) -> List[LidarFramePartition2D]: ...
    @property
    def is_valid(self) -> bool: ...
    def compute_closest_end_point(
        self, position: npt.NDArray[np.float64]
    ) -> TypedDict("returns", {"end_point_index": int, "distance": float}): ...
    @overload
    def sample_along_rays(
        self, num_samples_per_ray: int, max_in_obstacle_dist: float
    ) -> TypedDict(
        "returns",
        {
            "positions": npt.NDArray[np.float64],
            "directions": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    @overload
    def sample_along_rays(
        self, range_step: float, max_in_obstacle_dist: float
    ) -> TypedDict(
        "returns",
        {
            "positions": npt.NDArray[np.float64],
            "directions": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    def sample_near_surface(
        self, num_samples_per_ray: int, max_offset: float
    ) -> TypedDict(
        "returns",
        {
            "positions": npt.NDArray[np.float64],
            "directions": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    def sample_in_region(
        self, num_samples: int
    ) -> TypedDict(
        "returns",
        {
            "positions": npt.NDArray[np.float64],
            "directions": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
        },
    ): ...
    def compute_rays_at(
        self, position: npt.NDArray[np.float64]
    ) -> TypedDict(
        "returns",
        {
            "positions": npt.NDArray[np.float64],
            "directions": npt.NDArray[np.float64],
            "distances": npt.NDArray[np.float64],
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
    ) -> List[npt.NDArray[np.int32]]: ...

class CollisionCheckerBase:
    def is_collided(self: CollisionCheckerBase, grid_coords: npt.NDArray[np.int32]) -> bool: ...

class PointCollisionChecker2D(CollisionCheckerBase):
    def __init__(self: PointCollisionChecker2D, grid_map: GridMapUnsigned2D) -> None: ...

class PointCollisionChecker3D(CollisionCheckerBase):
    def __init__(self: PointCollisionChecker3D, grid_map: GridMapUnsigned3D) -> None: ...

class GridCollisionCheckerSe2:
    def __init__(
        self: GridCollisionCheckerSe2,
        grid_map: GridMapUnsigned2D,
        se2_grid_map_info: GridMapInfo3D,
        metric_shape: npt.NDArray[np.float64],
    ) -> None: ...
    @overload
    def is_collided(self: GridCollisionCheckerSe2, grid_coords: npt.NDArray[np.int32]) -> bool: ...
    @overload
    def is_collided(self: GridCollisionCheckerSe2, pose: npt.NDArray[np.float64]) -> bool: ...

class GridCollisionChecker3D:
    def __init__(
        self: GridCollisionChecker3D,
        grid_map: GridMapUnsigned3D,
        metric_voxels: npt.NDArray[np.float64],
    ) -> None: ...
    @overload
    def is_collided(self: GridCollisionCheckerSe2, grid_coords: npt.NDArray[np.int32]) -> bool: ...
    @overload
    def is_collided(self: GridCollisionCheckerSe2, pose: npt.NDArray[np.float64]) -> bool: ...
