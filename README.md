# About

This project is an attempt to fix `23_result.obj` a result produced by [tudelft3d/City3D](https://github.com/tudelft3d/City3D) with its default dataset

# How to use

* Run `build.sh` script to downlad CGAL, generate `CMakeLists.txt` and build the `cgal_orientation_fix` executable
* Run `./Release/cgal_orientation_fix` to verify that CGAL is not able to fix `23_result.obj`. By default the self-interecting faces are outputed for further inspection
* Create conda environment `conda env create -f environment.yml` and activate it with `conda activate city3d-result-check`
* Run `python3 clean_results_vertices_edges.py` to create a cleaned version of `23_result.obj`. This script merges very close vertices and add missing vertices to edges.
* Run `./Release/cgal_orientation_fix 23_result_cleaned.obj` to verify that CGAL is now able to produce `23_result_fixed.obj`, a clean City3D result with correct faces orientation. 
