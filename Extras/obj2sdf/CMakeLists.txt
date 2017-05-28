
SET(includes
  .
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
	${BULLET_PHYSICS_SOURCE_DIR}/src
)

LINK_LIBRARIES(
	 Bullet3Common
)

INCLUDE_DIRECTORIES(${includes})

ADD_EXECUTABLE(App_obj2sdf 
		obj2sdf.cpp
		../../examples/Utils/b3ResourcePath.cpp
		../../examples/Utils/b3ResourcePath.h
		../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
)

