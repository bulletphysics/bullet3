mkdir build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=ON -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release -DPYTHON_INCLUDE_DIR=c:\python38\include -DPYTHON_LIBRARY=c:\python38\libs\python38.lib -DPYTHON_DEBUG_LIBRARY=c:\python38\libs\python38.lib -G "Visual Studio 16 2019" ..
start .