
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)

LIST(APPEND SubDirList "CPU")


# Configure use of OpenCL and DX11
# Generates the settings file and defines libraries and include paths
OPTION(USE_OPENCL "Use OpenCL"	OFF)



if( USE_OPENCL )
	LIST(APPEND SubDirList "OpenCL")
endif( USE_OPENCL )
if( USE_DX11 )
	LIST(APPEND SubDirList "DX11")
endif( USE_DX11 )

SUBDIRS( ${SubDirList} )
