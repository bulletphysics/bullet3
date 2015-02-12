Name: bullet
Description: Bullet Continuous Collision Detection and Physics Library
Requires:
Version: @BULLET_VERSION@
Libs: -L@CMAKE_INSTALL_PREFIX@/@LIB_DESTINATION@ -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath
Cflags: @BULLET_DOUBLE_DEF@ -I@CMAKE_INSTALL_PREFIX@/@INCLUDE_INSTALL_DIR@ -I@CMAKE_INSTALL_PREFIX@/include
