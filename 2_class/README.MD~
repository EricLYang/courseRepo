# Second CLASS: 坐标系,运动,以及增量

#### Install Eigen 3
> For ubuntu user, just execute: sudo apt install libeigen3-dev

If not working, please download from [here](http://eigen.tuxfamily.org/index.php?title=Main_Page):

>git clone https://github.com/eigenteam/eigen-git-mirror.git

Then, in CMAKE file, add:

> SET( EIGEN3_INCLUDE ${yourproject_SOURCE_DIR}/eigen3 CACHE PATH "Directory of Eigen3")

Then, you should find eigen3 and link to your project.

#### eigen_example

In this demo project, it illustrate the basic operation of using eigen for pose processing and basic: dot, cross, inverse operation.

> - I have already include the eigen librarires in this folder: if you want to download by your own
> - download using: git clone https://github.com/eigenteam/eigen-git-mirror.git eigenLibrary

There is no need to compile eigen, just link to your project as: 
> - include_directories(${PROJECT_SOURCE_DIR}/eigenLibrary)

Then, under eigen_example folder:
> - cd build
> - rm -r *
> - cmake ..
> - make -j4

If you want to run the executable file:

> - ./eigenDemo

#### Install Sophus

1. Download the file
> git clone https://github.com/strasdat/Sophus.git

2. Install

> - mkdir build
> - cd build
> - cmake ..
> - make check (optional, runs unit tests)
> - make install

Here are some quite [example](https://github.com/strasdat/Sophus/tree/master/test/core)

3. For python users, please follow this [instruction](https://github.com/arntanguy/PySophus)

