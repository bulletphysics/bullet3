#!/bin/sh

if [ -e CMakeCache.txt ]; then
  rm CMakeCache.txt
fi
mkdir -p build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=ON -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release .. || exit 1
make -j $(command nproc 2>/dev/null || echo 12) || exit 1
cd examples
cd pybullet
if [ -e pybullet.dylib ]; then
  ln -f -s pybullet.dylib pybullet.so
fi
echo "Completed build of Bullet."
