prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=${prefix}
libdir=${exec_prefix}/@LIB_DESTINATION@
includedir=${prefix}/@INCLUDE_INSTALL_DIR@

Name: bullet
Description: Bullet Continuous Collision Detection and Physics Library
Version: @BULLET_VERSION@
Requires:
Libs: -L${libdir} -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath
Cflags: @BULLET_DOUBLE_DEF@ -I${includedir} -I${prefix}/include
