# lidar_visualization.py

# ==============================================================================
# Frameworks and Libraries Used
# ==============================================================================
# This script utilizes the following Python libraries:
#
# 1. Open3D: A modern library for 3D data processing. It's used here for
#    point cloud data structures, visualization, and geometric processing,
#    including clustering and bounding box creation.
#    - Website: http://www.open3d.org/
#
# 2. NumPy: The fundamental package for scientific computing in Python. It's
#    used for efficient array operations, particularly for handling the
#    point cloud data read from the binary file.
#    - Website: https://numpy.org/
# ==============================================================================

import open3d as o3d
import numpy as np
import struct

def read_point_cloud(file_path):
    """
    Reads a .bin file containing point cloud data.
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
    Main function to load, process, and visualize the point cloud data.
    """
    # ==============================================================================
    # Data Pre-processing
    # ==============================================================================
    # The pre-processing in this script involves the following steps:
    #
    # 1. Loading the Data:
    #    - The point cloud data is loaded from a binary file ('0000000001.bin').
    #    - We assume the data is stored as a sequence of 32-bit floats.
    #    - Each point consists of four floats: X, Y, Z coordinates, and an
    #      intensity value. The intensity value is ignored for this visualization.
    #
    # 2. Data Conversion:
    #    - The raw XYZ data (as a NumPy array) is converted into an Open3D
    #      PointCloud object. This is the primary data structure used by the
    #      Open3D library for any subsequent processing and visualization.
    # ==============================================================================

    # Load the point cloud data from the specified file
    file_path = '0000000001.bin'
    points = read_point_cloud(file_path)

    # Create an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # ==============================================================================
    # Point Cloud Clustering Model
    # ==============================================================================
    # For clustering the point cloud and identifying distinct objects, we use
    # the Density-Based Spatial Clustering of Applications with Noise (DBSCAN)
    # algorithm.
    #
    # - Model Used: open3d.geometry.PointCloud.cluster_dbscan
    #
    # - Why DBSCAN?
    #   - It does not require the number of clusters to be specified beforehand,
    #     which is ideal for surveillance applications where the number of
    #     objects is unknown and variable.
    #   - It can identify arbitrarily shaped clusters and is robust to noise
    #     (points that don't belong to any cluster).
    #
    # - Parameters:
    #   - eps (epsilon): The maximum distance between two points for one to be
    #     considered as in the neighborhood of the other. This parameter defines
    #     the density of clusters. We'll use a value of 0.5.
    #   - min_points: The minimum number of points required to form a dense
    #     region (a cluster). We'll set this to 10.
    # ==============================================================================

    # Perform DBSCAN clustering
    # You may need to tune 'eps' and 'min_points' for different datasets
    labels = np.array(pcd.cluster_dbscan(eps=0.5, min_points=10, print_progress=True))

    # Get the unique cluster labels, ignoring noise (-1)
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    # ==============================================================================
    # Object Detection and 3D Bounding Boxes
    # ==============================================================================
    # Object detection in this context is achieved by the clustering process.
    # Each cluster of points is considered a detected object.
    #
    # - Model Used: No pre-trained deep learning model is used. Object detection
    #   is based on the geometric clustering (DBSCAN) of the point cloud.
    #
    # - Bounding Box Generation:
    #   - For each cluster identified by DBSCAN, we generate an axis-aligned
    #     bounding box using `open3d.geometry.PointCloud.get_axis_aligned_bounding_box`.
    #   - These bounding boxes are then added to the list of geometries to be
    #     visualized alongside the point cloud.
    # ==============================================================================

    # Create a list to hold the bounding boxes for visualization
    bounding_boxes = []
    # Assign a unique color to each cluster for better visualization
    # We use a simple color map for this demonstration
    colors = np.random.rand(max_label + 1, 3)
    colors[0] = [0, 0, 0] # Noise points will be black if we decide to show them
    pcd.colors = o3d.utility.Vector3dVector(colors[labels])


    for i in range(max_label + 1):
        # Select points belonging to the current cluster
        cluster_indices = np.where(labels == i)[0]
        cluster_points = pcd.select_by_index(cluster_indices)

        if len(cluster_points.points) > 0:
            # Create an axis-aligned bounding box for the cluster
            bbox = cluster_points.get_axis_aligned_bounding_box()
            bbox.color = (1, 0, 0) # Set bounding box color to red
            bounding_boxes.append(bbox)

    # Visualize the point cloud with bounding boxes
    # The original point cloud 'pcd' is included to show all points
    o3d.visualization.draw_geometries([pcd] + bounding_boxes)

if __name__ == "__main__":
    main()
