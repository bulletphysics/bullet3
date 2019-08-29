prefix=@BULLET_INSTALL_PREFIX@
exec_prefix=${prefix}
libdir=${exec_prefix}/@BULLET_INSTALL_LIBDIR@
includedir=${prefix}/@BULLET_INSTALL_INCLUDEDIR@

Name: bullet
Description: Bullet Continuous Collision Detection and Physics Library
Version: @BULLET_VERSION@
Requires:
Libs: -L${libdir} -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath
Cflags: @BULLET_INSTALL_DEFINITIONS@ -I${includedir} -I${prefix}/@CMAKE_INSTALL_INCLUDEDIR@
