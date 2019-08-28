Description
===========
Individual tree segmentation in LiDAR-derived point clouds implemented using [The Point Cloud Library (PCL)](http://pointclouds.org/) and described in:
[Shendryk, I., M. Broich, M. G. Tulbure and S. V. Alexandrov (2016). "Bottom-up delineation of individual trees from full-waveform airborne laser scans in a structurally complex eucalypt forest." Remote Sensing of Environment 173: 69-83.](https://www.sciencedirect.com/science/article/pii/S0034425715301966)

and applied in:

[Shendryk, I., M. Broich, M. G. Tulbure, A. McGrath, D. Keith and S. V. Alexandrov (2016). "Mapping individual tree health using full-waveform airborne laser scans and imaging spectroscopy: A case study for a floodplain eucalypt forest." Remote Sensing of Environment 187: 202-217.](https://www.sciencedirect.com/science/article/pii/S0034425716303868)

[Shendryk, I., M. Broich and M. G. Tulbure (2018). "Multi-sensor airborne and satellite data for upscaling tree number information in a structurally complex eucalypt forest." International Journal of Applied Earth Observation and Geoinformation 73: 397-406.](https://www.sciencedirect.com/science/article/pii/S0303243418303155)

Acknowledgements
================
This work exists thanks to:
1) [Mirela Tulbure](https://scholar.google.com/citations?user=NHDv_PoAAAAJ&hl=en) 
2) [Mark Broich](https://scholar.google.com/citations?user=D2t2HsQAAAAJ&hl=en)
3) [Sergey Alexandrov](https://scholar.google.com/citations?user=uIZq6XsAAAAJ&hl=en)


Installation
============

1. Install universal pre-requisites (Ubuntu 16.04):

```bash
 sudo apt-get update
 sudo apt-get install git build-essential linux-libc-dev
 sudo apt-get install cmake cmake-gui 
 sudo apt-get install libeigen3-dev
 sudo apt-get install libboost-all-dev
 sudo apt-get install libflann-dev
 sudo apt-get install libvtk6-qt-dev
 sudo apt-get install libqhull-dev
 sudo apt-get install libproj-dev 

```

2. Install PCL v1.8.1 (Ubuntu 16.04):

```bash
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar -xf pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1 && mkdir build && cd build
cmake ..
make
sudo make install
```

3. Clone this repository (recursively) and make out-of-source build:

```bash
git clone --recursive https://github.com/yurithefury/ForestMetrics.git ForestMetrics
cd ForestMetrics
mkdir build
cd build
cmake ..
make
```

Data
====

There are examples of LiDAR scenes in .pcd format in the 'data/' folder. 
Use [las2pcd](https://github.com/murtiad/las2pcd) to convert .las to .pcd format.

Usage
=====

Navigate to the 'data/' folder and run

    ../bin/gui_delineation subset1.pcd

The program will load given file and proceed by building a graph of the input
point cloud. It will then display the graph (as a voxelized point cloud) so
that the user may select seed points. After the points are selected it will
perform random walker segmentation and visualize the results.

Visualizer interface
--------------------

This is the standard PCL visualizer with several extensions. It has a list of
objects available for visualization. To see it press `h`. The list will contain
status indicators, short descriptions, and keys that are used to toggle display
of the objects. For the random walker segmentation app it may look as follows:

                       Visualization objects
    ─────┬╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌┬──────
      ☐  │ Input point cloud                              │ i
      ☐  │ Graph vertices                                 │ v
      ☐  │ Vertex curvature                               │ C
      ☐  │ Vertex normals                                 │ n
      ☐  │ Adjacency edges                                │ a
      ☐  │ Random walker seeds                            │ S
      ☒  │ Object clusters                                │ c
    ─────┴╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌┴──────

For example, press `a` to toggle graph adjacency edges display.

Seed selection
--------------

Seed selection works as follows:

1. Hold shift and click a point. A randomly colored square will appear to confirm
   the selection. Select more points it you wish, they will all share the same
   label.
2. Press escape to start selecting points for the next label.
3. Repeat steps 1-2 until you selected point(s) for each object you want to
   segment in the scene.
4. Press espace once again.

Command-line options
--------------------

There are a number of command-line options, you may check it if you run the
program without passing any parameters. For example, you may try to change the
voxel resolution with `-v` option:

    ../bin/app_random_walker_segmentation forest.pcd -v 0.01

Another useful option is `--save`, which enables saving produced segmentation
into 'segmentation.pcd' file in the working directory. 

    
