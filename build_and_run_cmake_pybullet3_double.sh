#!/bin/sh
rm CMakeCache.txt
rm -r build_cmake_python3
mkdir build_cmake_python3
cd build_cmake_python3
case "$OSTYPE" in
  linux*)
    echo "Linux"
    cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release -DPYTHON_LIBRARY=/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu/libpython3.4.so -DPYTHON_INCLUDE_DIRS=/usr/include/python3.4m/ ..;;
  darwin*)
    echo "Mac"
    cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release -DPYTHON_INCLUDE_DIR=~/anaconda/envs/rllab3/include/python3.5m/ -DPYTHON_LIBRARY=~/anaconda/envs/rllab3/lib/libpython3.5m.dylib ..;;
esac
     
make -j8
