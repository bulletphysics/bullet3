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
	 BulletFileLoader  LinearMath
)

INCLUDE_DIRECTORIES(${includes})

SET(Main_LIBS  LinearMath)

ADD_EXECUTABLE(HeaderGenerator ${cpp_SRC} ${h_SRC})

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET HeaderGenerator
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/HeaderGenerator/createDnaString.bat ${CMAKE_CURRENT_BINARY_DIR}/createDnaString.bat
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/HeaderGenerator/bulletGenerate.py ${CMAKE_CURRENT_BINARY_DIR}/bulletGenerate.py
	)
ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
