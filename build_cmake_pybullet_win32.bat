mkdir cm
cd cm
cmake -DBUILD_PYBULLET=ON -DCMAKE_BUILD_TYPE=Release -DUSE_DOUBLE_PRECISION=ON -DPYTHON_INCLUDE_DIR=c:\python-3.5.2\include -DPYTHON_LIBRARY=c:\python-3.5.2\libs\python35.lib -DPYTHON_DEBUG_LIBRARY=c:\python-3.5.2\libs\python35_d.lib ..
start .
