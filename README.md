# Lidar Point Cloud Visualization with 3D Bounding Boxes

This project provides a Python script to visualize Lidar point cloud data. It processes a .bin file, identifies object clusters using the DBSCAN algorithm, and draws 3D axis-aligned bounding boxes around each detected cluster. This serves as a proof-of-concept for a Lidar-based surveillance solution.

## Features

*   Loads point cloud data from a binary .bin file.
    
*   Uses the **DBSCAN** algorithm via Open3D to cluster points into distinct objects.
    
*   Generates and displays 3D bounding boxes for each object.
    
*   Visualizes the colored point cloud and bounding boxes in an interactive 3D window.
    
## Tested Environment Specifications
This project was successfully tested on the following system configuration:

Operating System: Ubuntu 22.04.5 LTS

CPU: 11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz

## Setup and Installation

Follow these steps to set up the local development environment.

### Prerequisites

*   [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/products/distribution) should be installed.
    

### Installation Steps

1.  Clone the repository and change into the new directory: 
```
git clone https://github.com/elsontanjunhao/pcd_visual.git
cd pcd_visual
``` 
2.  Create and activate the Conda environment: 
```
conda create --name pcd_visual python=3.10
conda activate pcd_visual
```
3.  Install the required Python packages: 
```
pip install -r requirements.txt
```
4.  Update your system:
```
sudo apt update && sudo apt upgrade -y
```
5.  (Optional) Troubleshooting for Linux in Conda environment: A conflict can occur with the C++ library (libstdc++.so.6), which may prevent the visualization window from opening. To fix this for the current terminal session, run the following command before running the script:

To fix this for your current terminal session, run the following command **before** running the script:
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
```
Note that this command is temporary and must be run again if you open a new terminal.

## Usage

1.  Place your Lidar data file (e.g., `0000000001.bin`) in the root directory of the project.
    
2.  Ensure your `pcd_visual` Conda environment is activated.
    
3.  Run the main script from the terminal:
```
python lidar_visualization.py
```        
    
An Open3D window will open, displaying the pointcloud and the detected bounding boxes.

## Key Technologies Used

*   **Python 3.10**
    
*   **Open3D:** For 3D data processing, clustering, and visualization.
    
*   **NumPy:** For numerical operations on the point cloud data.
