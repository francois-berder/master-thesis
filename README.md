# Master thesis

Thesis for obtaining the Imperial College Master of Engineering in Computer Science.
Written between October 2014 and June 2015 with some additional fixes and improvements made over summer 2015.

This archive contains all necessary data to run the full program and produce results. I could not include full dataset because the archive would exceed size limit. Also, it does not contain all side programs that I wrote to generate graphs, test some features, preview some data...

UPDATE: The old folder contains most of the code I wrote during the project. Only feature7, stereo2 and algorithm2 were used.
feature7: apply feature point detection & matching on a pair of images.
stereo2: apply BM or SGBM to a pair of aligned images
algorithm2: old version of the semi-rigid stereo pipeline.

## Requirements

I use CMake to generate Makefiles. This project requires the following libraries:
  - OpenCV 2
  - OpenGV
  - gsl
  - liblbfgs
  - pthread
  - catch
Also, g++ must support C++11. I usually compile with g++ 4.9.2.

The python script requires Python 2.7 and the following libraries:
  - vtk
  - numpy
  - sys
  - csv

## Building instructions

To compile, run the following commands at the root of the archive:
cmake .
make

This creates two executables in the bin folder: meng_project and test. 

## Running program

Executables must always be run from the root of the archive.

```sh
./bin/meng_project <dataset_relative_path> [algorithms]
```

The program supports 3 algorithms:
  - RANSAC: `opengv-ransac<base_algo>-<nb_features>-<distance>`
  - Model: `model<reversed_index>`
  - Non-Linear minimization: `opengv-minimization<index>-<nb_features>-<distance>`

Beware of index parameter, the solution used will be solutions[solutions.size() - 1 - index]. For index=0, the algorithm will use the previous solution.
based-algo: [stewenius, nister, sevenpt, eightpt]

To view the point cloud:

```sh
python showcloud.py <relative_path_to_point_cloud_file>
```

### Example

```sh
./bin/meng_project data/shelves/ opengv-ransacstewenius-2000-40 model0 opengv-minimization0-2000-45
```

This command uses the shelves dataset and runs RANSAC, then rectifies the solution using the previous solution, minimizes the model rectified solution and finally computes the disparity map, the depth map and a point cloud.

All output are located in the output folder. The error file stores the reprojection error of each solution computed in the order they were computed for each entry of the dataset.
