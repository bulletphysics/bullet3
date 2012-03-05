INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}/include  )

SET(PfxUtil_SRCS
					pfx_mass.cpp
					pfx_mesh_creator.cpp
)

SET(PfxUtil_HDRS
					pfx_array.h
					pfx_array_implementation.h
					pfx_util_common.h
)




ADD_LIBRARY(PfxUtil ${PfxUtil_SRCS} ${PfxUtil_HDRS})

SET_TARGET_PROPERTIES(PfxUtil PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(PfxUtil PROPERTIES SOVERSION ${BULLET_VERSION})
