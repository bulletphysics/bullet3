INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/Extras/iff
)

ADD_LIBRARY(Iff
iff.h 
iffr.cpp
iffw.cpp
)

#SUBDIRS( BulletIffConverter )
