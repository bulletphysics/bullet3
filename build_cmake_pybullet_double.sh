#!/bin/sh

if [ -e CMakeCache.txt ]; then
  rm CMakeCache.txt
fi
mkdir -p build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release ..
make -j $(command nproc || echo 12)
cd examples
cd pybullet
if [ -e pybullet.dylib ]; then
  rm pybullet.so
  ln -s pybullet.dylib pybullet.so
fi

