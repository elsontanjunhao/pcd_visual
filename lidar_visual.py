# lidar_visual.py

# ==============================================================================
# Frameworks and Libraries Used
# ==============================================================================
# This script utilizes the following Python libraries:
#
# 1. Open3D: A modern library for 3D data processing. It's used here for
#    pointcloud data structures, visualization, and geometric processing,
#    including clustering and bounding box creation.
#
# 2. NumPy: The fundamental package for scientific computing in Python. It's
#    used for efficient array operations, particularly for handling the
#    pointcloud data read from the binary file.
# ==============================================================================

import open3d as o3d
import numpy as np
import struct

def read_pointcloud(file_path):
    """
    Reads a .bin file containing pointcloud data.
    The data is expected to be in XYZI (x, y, z, intensity) format.
    The intensity value is discarded, and only XYZ is returned.
    """
    points = []
    with open(file_path, 'rb') as f:
        while True:
            # Read 4 floats (4 bytes each) for each point (x, y, z, intensity)
            point_data = f.read(16)
            if not point_data:
                break
            # Unpack the binary data into four floats
            x, y, z, _ = struct.unpack('ffff', point_data)
            points.append([x, y, z])
    return np.array(points)

def main():
    """
    Main function to load, process, and visualize the pointcloud data.
    """
    # ==============================================================================
    # Configuration Parameters
    # ==============================================================================
    # --- Input File ---
    file_path = '0000000001.bin'

    # --- Pre-processing Parameters ---
    distance_crop_threshold = 50.0      # Max distance in meters from center to keep points.
    voxel_size = 0.02                    # Voxel size for downsampling in meters.
    stat_outlier_num_neighbors = 20     # Number of neighbors for statistical outlier removal.
    stat_outlier_std_ratio = 2.0        # Standard deviation ratio for statistical outlier removal.
    
    # --- Ground Plane (RANSAC) Parameters ---
    ransac_distance_threshold = 0.25     # Max distance a point can be from the plane model.
    ransac_n = 3                        # Number of points to sample for a plane.
    ransac_num_iterations = 5000        # Number of RANSAC iterations.

    # --- Clustering (DBSCAN) Parameters ---
    dbscan_eps = 0.5                    # Epsilon value (neighborhood distance).
    dbscan_min_points = 5               # Minimum points to form a cluster.
    # ==============================================================================

    # ==============================================================================
    # Data Pre-processing
    # ==============================================================================
    # The pre-processing in this script involves the following steps:
    #
    # 1. Loading the Data:
    #    - The pointcloud data is loaded from the binary file specified above.
    #
    # 2. Distance Cropping:
    #    - Points that are excessively far from the center of the entire scene
    #      are removed to focus processing on the area of interest.
    #
    # 3. Voxel Downsampling:
    #    - The point cloud is downsampled to reduce the number of points and
    #      create a more uniform point density.
    #
    # 4. Statistical Outlier Removal:
    #    - Sparse outliers are removed to clean up noise from sensor errors.
    #
    # 5. Ground Plane Removal:
    #    - The RANSAC algorithm is used to identify and remove the ground plane.
    # ==============================================================================

    points = read_pointcloud(file_path)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # --- Crop points that are too far from the pointcloud center ---
    pcd_center = pcd.get_center()
    distances = np.linalg.norm(np.asarray(pcd.points) - pcd_center, axis=1)
    nearby_indices = np.where(distances < distance_crop_threshold)[0]
    pcd_cropped = pcd.select_by_index(nearby_indices)

    # --- Downsample the cropped pointcloud ---
    pcd_down = pcd_cropped.voxel_down_sample(voxel_size)
    
    # --- Remove sparse outliers from the downsampled pointcloud ---
    pcd_filtered, _ = pcd_down.remove_statistical_outlier(
        nb_neighbors=stat_outlier_num_neighbors,
        std_ratio=stat_outlier_std_ratio
    )

    # --- Ground Plane Removal using RANSAC ---
    plane_model, inlier_indices = pcd_filtered.segment_plane(
        distance_threshold=ransac_distance_threshold,
        ransac_n=ransac_n,
        num_iterations=ransac_num_iterations
    )
    
    pcd_objects = pcd_filtered.select_by_index(inlier_indices, invert=True)
    ground_plane = pcd_filtered.select_by_index(inlier_indices)
    ground_plane.paint_uniform_color([0.5, 0.5, 0.5])

    # ==============================================================================
    # PointCloud Clustering Model
    # ==============================================================================
    # Using the Density-Based Spatial Clustering of Applications with Noise (DBSCAN)
    # algorithm to identify distinct objects in the scene.
    # ==============================================================================

    labels = np.array(pcd_objects.cluster_dbscan(
        eps=dbscan_eps, 
        min_points=dbscan_min_points, 
        print_progress=True
    ))

    max_label = labels.max()
    print(f"Pointcloud has {max_label + 1} clusters")

    # ==============================================================================
    # Object Detection and 3D Bounding Boxes
    # ==============================================================================
    # - Model Used: No pre-trained deep learning object detection model is used.
    #
    # In this script, "object detection" is achieved by treating each cluster
    # found by the DBSCAN algorithm as a distinct object. Bounding boxes are then
    # generated for each of these clusters.
    # ==============================================================================

    colors = np.random.rand(max_label + 2, 3)
    colors[-1] = [0, 0, 0]
    pcd_objects.colors = o3d.utility.Vector3dVector(colors[labels])

    bounding_boxes = []
    for i in range(max_label + 1):
        cluster_indices = np.where(labels == i)[0]
        if len(cluster_indices) > 0:
            cluster_points = pcd_objects.select_by_index(cluster_indices)
            bbox = cluster_points.get_axis_aligned_bounding_box()
            bbox.color = (1, 0, 0)
            bounding_boxes.append(bbox)

    # Visualize the scene
    o3d.visualization.draw_geometries([ground_plane, pcd_objects] + bounding_boxes)

if __name__ == "__main__":
    main()

