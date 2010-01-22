###############################################################################
PROJECT(GEN)
FILE(GLOB cpp_SRC "*.cpp")
FILE(GLOB h_SRC "*.h")


SET(includes 
  .
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BlenderSerialize
	${BULLET_PHYSICS_SOURCE_DIR}/src
)


LINK_LIBRARIES(
	 BulletFileLoader BlenderSerialize LinearMath
)

INCLUDE_DIRECTORIES(${includes})

SET(Main_LIBS  LinearMath)

ADD_EXECUTABLE(HeaderGenerator ${cpp_SRC} ${h_SRC})
