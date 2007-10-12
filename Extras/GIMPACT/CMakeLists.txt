INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/Extras/GIMPACT/include
${BULLET_PHYSICS_SOURCE_DIR}/src
)

ADD_LIBRARY(LibGIMPACT
		src/Bullet/btContactProcessing.cpp
		src/Bullet/btGImpactBvh.cpp
		src/Bullet/btGImpactQuantizedBvh.cpp
		src/Bullet/btTriangleShapeEx.cpp
		src/Bullet/btGImpactCollisionAlgorithm.cpp
		src/Bullet/btGImpactShape.cpp
		src/core/gim_box_set.cpp
		src/core/gim_contact.cpp
		src/core/gim_memory.cpp
		src/core/gim_tri_collision.cpp
)
