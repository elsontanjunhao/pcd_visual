Lidar Point Cloud Visualization with 3D Bounding Boxes
This project provides a Python script to visualize Lidar point cloud data. It processes a .bin file, identifies object clusters using the DBSCAN algorithm, and draws 3D axis-aligned bounding boxes around each detected cluster. This serves as a proof-of-concept for a Lidar-based surveillance solution.

Features
Loads point cloud data from a binary .bin file.

Uses the DBSCAN algorithm via Open3D to cluster points into distinct objects.

Generates and displays 3D bounding boxes for each object.

Visualizes the colored point cloud and bounding boxes in an interactive 3D window.

Setup and Installation
Follow these steps to set up the local development environment.

Prerequisites
Miniconda or Anaconda must be installed.

Installation Steps
Clone the repository:

git clone [https://github.com/elson-tan/pcd_visual.git](https://github.com/elson-tan/pcd_visual.git)
cd pcd_visual

Create and activate the Conda environment:

conda create --name pcd_visual python=3.10
conda activate pcd_visual

Install the required Python packages:

pip install -r requirements.txt

Usage
Place your Lidar data file (e.g., 0000000001.bin) in the root directory of the project.

Ensure your pcd_visual Conda environment is activated.

Run the main script from the terminal:

python lidar_visualization.py

An Open3D window will open, displaying the point cloud and the detected bounding boxes.

Key Technologies Used
Python 3.10

Open3D: For 3D data processing, clustering, and visualization.

NumPy: For numerical operations on the point cloud data.
