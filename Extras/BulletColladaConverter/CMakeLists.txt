INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexHull ${BULLET_PHYSICS_SOURCE_DIR}/Extras/BulletColladaConverter  ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML/include ${BULLET_PHYSICS_SOURCE_DIR}/Extras/COLLADA_DOM/include ${BULLET_PHYSICS_SOURCE_DIR}/Extras/COLLADA_DOM/include/1.4
)

ADD_LIBRARY(LibBulletColladaConverter
			ColladaConverter.h
			ColladaConverter.cpp
)
