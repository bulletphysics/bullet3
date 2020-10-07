cd build_cmake_debug
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS -DBUILD_UNIT_TESTS ..
make -j $(command nproc 2>/dev/null || echo 12) || exit 1
cd -
# Copy .so files to ./lib/Release
# See here: https://github.com/Kitware/CMake/blob/master/Modules/FindBullet.cmake
rm -rf ./lib
mkdir ./lib
mkdir ./lib/Release
cp ./build_cmake_debug/src/BulletDynamics/libBulletDynamics.so* ./lib/Release/
cp ./build_cmake_debug/src/BulletCollision/libBulletCollision.so* ./lib/Release/
cp ./build_cmake_debug/src/BulletSoftBody/libBulletSoftBody.so* ./lib/Release/
cp ./build_cmake_debug/src/LinearMath/libLinearMath.so* ./lib/Release/
