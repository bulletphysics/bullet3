Name: bullet_robotics
Description: Bullet extras that include several utilities for robotics including a urdf parser
Requires: bullet
Version: @BULLET_VERSION@
Libs: -L@CMAKE_INSTALL_PREFIX@/@LIB_DESTINATION@ -lBulletRobotics
Cflags: @BULLET_DOUBLE_DEF@ -I@CMAKE_INSTALL_PREFIX@/@INCLUDE_INSTALL_DIR@ -I@CMAKE_INSTALL_PREFIX@/include/bullet_robotics
