# 24783 Group Project - NamingSoHard

##Alpha version

Note: pcd_viewer is a ros package, independent of all other codes

## Final/Beta version

### Dependency

Eigen3 is required to build this project.

On Linux systems:

```bash
sudo apt install libeigen3-dev
```

On MacOS:
https://stackoverflow.com/questions/35658420/installing-eigen-on-mac-os-x-for-xcode

On windows:
Download the desired release from http://eigen.tuxfamily.org.
Unzip in the location of your choice, preferrably at C:\ or C:\Program files for better discoverability by CMake find-modules (remember to extract the inner folder and rename it to Eigen3 or Eigen).

### Build

Clone the repo and create build directory, the structure should look like:

24783
└─build
└─public
└─NamingSoHard

Inside build directory:

```bash
cmake ../NamingSoHard/codebase/Beta  -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

To run tests

```bash
ctest
```

To visualize examples:

```bash
./find_ws ../NamingSoHard/codebase/Beta/tests/3dof.urdf 20 8
```

where the first argument is the directory to the urdf file, the second argument is the linespace interval of the joint angles, and the last argument is the number of threads to use.
