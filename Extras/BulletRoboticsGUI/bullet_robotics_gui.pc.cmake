Name: bullet_robotics_gui
Description: Bullet GUI extras for robotics
Requires: bullet
Version: @BULLET_VERSION@
Libs: -L@CMAKE_INSTALL_PREFIX@/@LIB_DESTINATION@ -lBulletRoboticsGUI
Cflags: @BULLET_DOUBLE_DEF@ -I@CMAKE_INSTALL_PREFIX@/@INCLUDE_INSTALL_DIR@ -I@CMAKE_INSTALL_PREFIX@/include/bullet_robotics_gui
