prefix=@BULLET_INSTALL_PREFIX@
exec_prefix=${prefix}
libdir=${exec_prefix}/bullet_robotics
includedir=${prefix}/@CMAKE_INSTALL_INCLUDEDIR@/bullet_robotics

Name: bullet_robotics
Description: Bullet extras that include several utilities for robotics including a urdf parser
Version: @BULLET_VERSION@
Requires: bullet
Libs: -L${libdir} -lBulletRobotics
Cflags: @BULLET_DOUBLE_DEF@ -I${includedir} -I@{prefix}/@BULLET_INSTALL_INCLUDEDIR@ -I@CMAKE_INSTALL_INCLUDEDIR@
