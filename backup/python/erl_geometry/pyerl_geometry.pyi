from enum import IntEnum
from typing import Callable
from typing import Optional
from typing import Tuple
from typing import overload

import numpy as np
import numpy.typing as npt

from erl_common.storage import GridMapInfo2D
from erl_common.storage import GridMapInfo3D
from erl_common.storage import GridMapUnsigned2D
from erl_common.storage import GridMapUnsigned3D
from erl_common.yaml import YamlableBase

class Node:
    def __init__(self: Node, type: int, position: np.ndarray[np.float64], node_data: NodeData = None): ...

    @property
    def position(self: Node) -> np.ndarray[np.float64]: ...

    @property
    def type(self: Node) -> int: ...

    @property
    def node_data(self: Node) -> NodeData: ...


class NodeData:
    def __str__(self) -> str: ...


class NodeContainer:
    @property
    def node_types(self: NodeContainer) -> list[int]: ...

    def node_type_name(self: NodeContainer, type: int) -> str: ...

    def size(self: NodeContainer) -> int: ...

    def size_of_type(self: NodeContainer, type: int) -> int: ...

    def empty(self: NodeContainer) -> bool: ...

    def empty_of_type(self: NodeContainer, type: int) -> bool: ...

    def clear(self: NodeContainer) -> None: ...

    def collect_nodes(self: NodeContainer) -> list[Node]: ...

    def collect_nodes_of_type(self: NodeContainer, type: int) -> list[Node]: ...

    def collect_nodes_of_type_in_aabb_2d(self: NodeContainer, type: int, aabb_2d: Aabb2D) -> list[Node]: ...

    def collect_nodes_of_type_in_aabb_3d(self: NodeContainer, type: int, aabb_3d: Aabb3D) -> list[Node]: ...

    def insert(self: NodeContainer, node: Node) -> bool: ...

    def remove(self: NodeContainer, node: Node) -> bool: ...


class NodeContainerMultiTypes(NodeContainer):
    class Setting(YamlableBase):
        num_node_types: int
        node_type_capacity: list[int]
        node_type_min_squared_distance: list[float]

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
    ) -> list[IncrementalQuadtree]: ...

    def collect_non_empty_clusters(self: IncrementalQuadtree, area: Aabb2D) -> list[IncrementalQuadtree]: ...

    def collect_nodes(self: IncrementalQuadtree) -> list[Node]: ...

    def collect_nodes_of_type(self: IncrementalQuadtree, type: int) -> list[Node]: ...

    def collect_nodes_of_type_in_area(self: IncrementalQuadtree, type: int, area: Aabb2D) -> list[Node]: ...

    @property
    def node_types(self: IncrementalQuadtree): ...

    @overload
    def ray_tracing(
            self: IncrementalQuadtree,
            ray_origin: np.ndarray[np.float64],
            ray_direction: np.ndarray[np.float64],
            hit_distance_threshold: float,
    ) -> Tuple[float, Node]: ...

    @overload
    def ray_tracing(
            self: IncrementalQuadtree,
            ray_origins: np.ndarray[np.float64],
            ray_directions: np.ndarray[np.float64],
            hit_distance_threshold: float,
    ) -> Tuple[list[float], list[Node]]: ...

    @overload
    def ray_tracing(
            self: IncrementalQuadtree,
            node_type: int,
            ray_origin: np.ndarray[np.float64],
            ray_direction: np.ndarray[np.float64],
            hit_distance_threshold: float,
    ) -> Tuple[float, Node]: ...

    @overload
    def ray_tracing(
            self: IncrementalQuadtree,
            node_type: int,
            ray_origins: np.ndarray[np.float64],
            ray_directions: np.ndarray[np.float64],
            hit_distance_threshold: float,
    ) -> Tuple[list[float], list[Node]]: ...

    def plot(
            self: IncrementalQuadtree,
            grid_map_info: GridMapInfo2D,
            node_types: list[int],
            node_type_colors: Dict[int, np.ndarray[np.int32]],
            node_type_radius: Dict[int, int],
            bg_color: np.ndarray[np.float64] = np.array([255, 255, 255]),
            area_rect_color: np.ndarray[np.float64] = np.array([0, 0, 0]),
            area_rect_thickness: int = 2,
            tree_data_color: np.ndarray[np.float64] = np.array([255, 0, 0]),
            tree_data_radius: int = 2,
    ) -> np.ndarray[np.uint8]: ...

    def __str__(self: IncrementalQuadtree) -> str: ...


class CollisionCheckerBase:
    def is_collided(self: CollisionCheckerBase, grid_coords: np.ndarray[np.int32]) -> bool: ...


class PointCollisionChecker2D(CollisionCheckerBase):
    def __init__(self: PointCollisionChecker2D, grid_map: GridMapUnsigned2D) -> None: ...


class PointCollisionChecker3D(CollisionCheckerBase):
    def __init__(self: PointCollisionChecker3D, grid_map: GridMapUnsigned3D) -> None: ...


class GridCollisionCheckerSe2:
    def __init__(
            self: GridCollisionCheckerSe2,
            grid_map: GridMapUnsigned2D,
            se2_grid_map_info: GridMapInfo3D,
            metric_shape: np.ndarray[np.float64],
    ) -> None: ...

    @overload
    def is_collided(self: GridCollisionCheckerSe2, grid_coords: np.ndarray[np.int32]) -> bool: ...

    @overload
    def is_collided(self: GridCollisionCheckerSe2, pose: np.ndarray[np.float64]) -> bool: ...


class GridCollisionChecker3D:
    def __init__(
            self: GridCollisionChecker3D,
            grid_map: GridMapUnsigned3D,
            metric_voxels: np.ndarray[np.float64],
    ) -> None: ...

    @overload
    def is_collided(self: GridCollisionCheckerSe2, grid_coords: np.ndarray[np.int32]) -> bool: ...

    @overload
    def is_collided(self: GridCollisionCheckerSe2, pose: np.ndarray[np.float64]) -> bool: ...
