Standalone CMake Project using Bullet Simulator
===============================================

This will build the bullet3 RobotSimulator example,
uisng a standalone CMakeLists.txt that links to the
Bullet libraries and relevant examples libraries.

In this example, bullet3 is installed to `~/Downloads/install`.


Build Bullet3
-------------

    cd bullet3
    mkdir -p build && cd build
    cmake .. \
      -DUSE_DOUBLE_PRECISION=ON \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=~/Downloads/install
    make -j3
    make install

Build RobotSimulator standalone
-------------------------------

    cd bullet3/StandaloneCmakeProject
    mkdir -p build && d build
    cmake .. -DCMAKE_PREFIX_PATH=~/Downloads/install
    make -j3

Run RobotSimulator
------------------

    LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Downloads/install/lib
    ./RobotSimulator
