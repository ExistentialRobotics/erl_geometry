#!/usr/bin/env python3
"""
Point Cloud to Mesh Reconstruction Script

This script reconstructs surface meshes from point clouds using PyMeshLab.
Supports two reconstruction methods:
1. Poisson Surface Reconstruction
2. Ball Pivoting Algorithm

Author: Zhirui Dai
Email: daizhirui@hotmail.com
License: MIT

Reference:
1. https://gist.github.com/shubhamwagh/0dc3b8173f662d39d4bf6f53d0f4d66b?permalink_comment_id=3972755
2. https://pymeshlab.readthedocs.io/en/latest/filter_list.html
"""

import argparse
import sys
import os
import numpy as np
import pymeshlab


def load_point_cloud(input_file):
    """
    Load point cloud from file using PyMeshLab.

    Args:
        input_file (str): Path to input point cloud file

    Returns:
        pymeshlab.MeshSet: Loaded mesh set containing the point cloud
    """
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    ms = pymeshlab.MeshSet()
    try:
        ms.load_new_mesh(input_file)
        print(f"Loaded point cloud: {input_file}")
        print(f"Number of vertices: {ms.current_mesh().vertex_number()}")
        print(f"Number of faces: {ms.current_mesh().face_number()}")
        return ms
    except Exception as e:
        raise RuntimeError(f"Failed to load point cloud: {e}")


def estimate_normals(
    ms,
    ignore_existing_normals=False,
    k_neighbors=10,
    smooth_iterations=0,
    flip_normals=False,
    view_position=None,
):
    """
    Estimate normals for the point cloud if not present.

    Args:
        ms (pymeshlab.MeshSet): Mesh set containing the point cloud
        k_neighbors (int): Number of neighbors for normal estimation
        smooth_iterations (int): Number of smoothing iterations
    """
    try:
        # Check if normals already exist
        if (ms.current_mesh().vertex_normal_matrix() > 0).any() and not ignore_existing_normals:
            print("Point cloud already has normals")
            return

        print(f"Estimating normals using {k_neighbors} neighbors...")
        if view_position is None:
            view_position = ms.current_mesh().bounding_box().center()
        ms.compute_normal_for_point_clouds(
            k=k_neighbors,
            smoothiter=smooth_iterations,
            flipflag=flip_normals,
            viewpos=view_position,
        )
        print("Normal estimation completed")
    except Exception as e:
        print(f"Warning: Failed to estimate normals: {e}")
        exit(1)


def poisson_reconstruction(ms, depth=8, point_weight=4.0, samples_per_node=1.5):
    """
    Perform Poisson surface reconstruction.

    Args:
        ms (pymeshlab.MeshSet): Mesh set containing the point cloud
        depth (int): Maximum depth of the octree (default: 8)
        point_weight (float): Importance of point interpolation (default: 4.0)
        samples_per_node (float): Minimum samples per octree node (default: 1.5)
    """
    try:
        print("Starting Poisson surface reconstruction...")
        print(f"Parameters: depth={depth}, point_weight={point_weight}, samples_per_node={samples_per_node}")

        ms.generate_surface_reconstruction_screened_poisson(
            depth=depth, pointweight=point_weight, samplespernode=samples_per_node, threads=os.cpu_count()
        )
        ms.set_current_mesh(ms.mesh_number() - 1)  # Set the reconstructed mesh as current

        print("Poisson reconstruction completed")
        print(
            f"Generated mesh - Vertices: {ms.current_mesh().vertex_number()}, "
            f"Faces: {ms.current_mesh().face_number()}"
        )

    except Exception as e:
        raise RuntimeError(f"Poisson reconstruction failed: {e}")


def ball_pivoting_reconstruction(ms, radius_multiplier=1.0, clustering=0.1, angle_threshold=90.0):
    """
    Perform Ball Pivoting surface reconstruction.

    Args:
        ms (pymeshlab.MeshSet): Mesh set containing the point cloud
        radius_multiplier (float): Radius multiplier for ball size (default: 1.0)
        clustering (float): Clustering radius (default: 0.1)
        angle_threshold (float): Angle threshold in degrees (default: 90.0)
    """
    try:
        print("Starting Ball Pivoting surface reconstruction...")

        # Estimate ball radius based on point cloud
        bbox = ms.current_mesh().bounding_box()
        diag = np.linalg.norm([bbox.max()[i] - bbox.min()[i] for i in range(3)])
        radius = diag * radius_multiplier / 100.0

        print(f"Parameters: radius={radius:.6f}, clustering={clustering}, angle_threshold={angle_threshold}")

        ms.generate_surface_reconstruction_ball_pivoting(
            ballradius=pymeshlab.PercentageValue(radius_multiplier),
            clustering=clustering * 100.0,  # Convert to percentage
            creasethr=np.radians(angle_threshold),
        )
        ms.set_current_mesh(ms.mesh_number() - 1)  # Set the reconstructed mesh as current

        print("Ball Pivoting reconstruction completed")
        print(
            f"Generated mesh - Vertices: {ms.current_mesh().vertex_number()},"
            f"Faces: {ms.current_mesh().face_number()}"
        )

    except Exception as e:
        raise RuntimeError(f"Ball Pivoting reconstruction failed: {e}")


def post_process_mesh(
    ms,
    remove_long_edges=1.0,
    remove_isolated=True,
    remove_duplicates=True,
    remove_non_manifold=True,
    smooth_iterations=0,
    pause=False,
):
    """
    Post-process the reconstructed mesh.

    Args:
        ms (pymeshlab.MeshSet): Mesh set containing the reconstructed mesh
        remove_long_edges (float): Threshold for removing edges longer than this value
        remove_isolated (bool): Remove isolated vertices and faces
        remove_duplicates (bool): Remove duplicate vertices and faces
        remove_non_manifold (bool): Remove non-manifold edges
        smooth_iterations (int): Number of Laplacian smoothing iterations
    """
    try:
        if remove_long_edges > 0:
            print(f"Removing edges longer than {remove_long_edges}...")
            ms.apply_filter("compute_selection_by_edge_length", threshold=remove_long_edges)
            ms.meshing_remove_selected_vertices_and_faces()
            pause_if_needed(pause, ms)

        if remove_duplicates:
            print("Removing duplicate vertices...")
            ms.meshing_remove_duplicate_vertices()
            pause_if_needed(pause, ms)

        if remove_isolated:
            print("Removing isolated components...")
            ms.meshing_remove_connected_component_by_face_number(mincomponentsize=10)
            pause_if_needed(pause, ms)

        if remove_non_manifold:
            print("Removing non-manifold edges...")
            ms.apply_filter("compute_selection_by_non_manifold_edges_per_face")
            ms.meshing_remove_selected_vertices_and_faces()
            pause_if_needed(pause, ms)

        if smooth_iterations > 0:
            print(f"Applying {smooth_iterations} iterations of Laplacian smoothing...")
            ms.apply_coord_laplacian_smoothing(stepsmoothnum=smooth_iterations)
            pause_if_needed(pause, ms)

        print("Post-processing completed")
        print(
            f"Final mesh - Vertices: {ms.current_mesh().vertex_number()},",
            f"Faces: {ms.current_mesh().face_number()}",
        )

    except Exception as e:
        print(f"Warning: Post-processing failed: {e}")


def save_mesh(ms, output_file):
    """
    Save the reconstructed mesh to file.

    Args:
        ms (pymeshlab.MeshSet): Mesh set containing the reconstructed mesh
        output_file (str): Path to output mesh file
    """
    try:
        # Create output directory if it doesn't exist
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        ms.save_current_mesh(output_file)
        print(f"Mesh saved to: {output_file}")
    except Exception as e:
        raise RuntimeError(f"Failed to save mesh: {e}")


def pause_if_needed(pause, ms):
    if pause:
        print("Close the mesh viewer to continue...")
        ms.show_polyscope()


def main():
    """Main function to handle command line arguments and execute reconstruction."""
    parser = argparse.ArgumentParser(
        description="Reconstruct surface mesh from point cloud using PyMeshLab",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Poisson reconstruction with default parameters
  python point_cloud_to_mesh.py input.ply output.obj --method poisson

  # Ball Pivoting with custom parameters
  python point_cloud_to_mesh.py input.pcd output.ply --method ball_pivot --radius 2.0

  # Poisson with custom depth and post-processing
  python point_cloud_to_mesh.py input.xyz output.stl --method poisson --depth 10 --smooth 3
        """,
    )

    # Required arguments
    parser.add_argument("input", help="Input point cloud file (PLY, PCD, XYZ, OBJ, etc.)")
    parser.add_argument("output", help="Output mesh file (PLY, OBJ, STL, etc.)")

    # Method selection
    parser.add_argument(
        "--method",
        choices=["poisson", "ball_pivot"],
        default="poisson",
        help="Reconstruction method (default: poisson)",
    )

    # Point cloud downsampling parameters
    parser.add_argument(
        "--downsample",
        type=int,
        default=1,
        help="Downsample point cloud by this factor (default: 1)",
    )

    # Normal estimation parameters
    parser.add_argument(
        "--ignore-existing-normals",
        action="store_true",
        help="Ignore existing normals and re-estimate them",
    )
    parser.add_argument(
        "--no-flip-normals",
        action="store_false",
        dest="flip_normals",
        help="Don't flip normals according to the view position",
    )
    parser.add_argument(
        "--view-position",
        type=float,
        nargs=3,
        default=None,
        help="View position for normal estimation (default: center of bounding box)",
    )
    parser.add_argument(
        "--k-neighbors",
        type=int,
        default=10,
        help="Number of neighbors for normal estimation (default: 10)",
    )
    parser.add_argument("--normal-smooth", type=int, default=0, help="Normal smoothing iterations (default: 0)")

    # Poisson reconstruction parameters
    parser.add_argument("--depth", type=int, default=8, help="Poisson octree depth (default: 8)")
    parser.add_argument(
        "--point-weight", type=float, default=4.0, help="Poisson point interpolation weight (default: 4.0)"
    )
    parser.add_argument(
        "--samples-per-node", type=float, default=1.5, help="Poisson samples per octree node (default: 1.5)"
    )

    # Ball Pivoting parameters
    parser.add_argument("--radius", type=float, default=10.0, help="Ball Pivoting radius multiplier (default: 10.0%)")
    parser.add_argument("--clustering", type=float, default=0.1, help="Ball Pivoting clustering radius (default: 0.1)")
    parser.add_argument(
        "--angle-threshold",
        type=float,
        default=90.0,
        help="Ball Pivoting angle threshold in degrees (default: 90.0)",
    )

    # Post-processing options
    parser.add_argument(
        "--remove-long-edges",
        type=float,
        default=1.0,
        help="Remove edges longer than this value (default: 1.0)",
    )
    parser.add_argument(
        "--no-remove-isolated",
        action="store_false",
        dest="remove_isolated",
        help="Don't remove isolated components",
    )
    parser.add_argument(
        "--no-remove-duplicates",
        action="store_false",
        dest="remove_duplicates",
        help="Don't remove duplicate vertices",
    )
    parser.add_argument(
        "--no-remove-non-manifold",
        action="store_false",
        dest="remove_non_manifold",
        help="Don't remove non-manifold edges",
    )
    parser.add_argument("--smooth", type=int, default=0, help="Number of Laplacian smoothing iterations (default: 0)")

    # Other options
    parser.add_argument("--pause", action="store_true", help="Pause after each step for debugging")

    args = parser.parse_args()

    try:
        # Load point cloud
        ms = load_point_cloud(args.input)
        pause_if_needed(args.pause, ms)

        # Downsample point cloud if needed
        if args.downsample > 1:
            print(f"Downsampling point cloud by factor {args.downsample}...")
            n = ms.current_mesh().vertex_number()
            ms.apply_filter("generate_simplified_point_cloud", samplenum=n // args.downsample)
            print(f"Downsampled to {ms.current_mesh().vertex_number()} vertices")
            ms.set_current_mesh(ms.mesh_number() - 1)
            pause_if_needed(args.pause, ms)

        # Estimate normals if needed
        estimate_normals(
            ms,
            args.ignore_existing_normals,
            args.k_neighbors,
            args.normal_smooth,
            args.flip_normals,
            args.view_position,
        )
        pause_if_needed(args.pause, ms)

        # Perform reconstruction based on selected method
        if args.method == "poisson":
            poisson_reconstruction(ms, args.depth, args.point_weight, args.samples_per_node)
        elif args.method == "ball_pivot":
            ball_pivoting_reconstruction(ms, args.radius, args.clustering, args.angle_threshold)
        pause_if_needed(args.pause, ms)

        # Post-process mesh
        post_process_mesh(
            ms,
            remove_long_edges=args.remove_long_edges,
            remove_isolated=args.remove_isolated,
            remove_duplicates=args.remove_duplicates,
            remove_non_manifold=args.remove_non_manifold,
            smooth_iterations=args.smooth,
            pause=args.pause,
        )

        # Save the result
        save_mesh(ms, args.output)

        print("Surface reconstruction completed successfully!")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
