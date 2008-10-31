INCLUDE_DIRECTORIES(
 ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexDecomposition ${BULLET_PHYSICS_SOURCE_DIR}/src
)
	
ADD_LIBRARY(LibConvexDecomposition
	ConvexDecomposition.h	bestfitobb.cpp		cd_vector.h		concavity.h		float_math.h		planetri.h		splitplane.h
ConvexBuilder.cpp		bestfitobb.h		cd_wavefront.cpp	fitsphere.cpp		meshvolume.cpp		raytri.cpp		vlookup.cpp
ConvexBuilder.h		bestfit.cpp		cd_hull.cpp		cd_wavefront.h		fitsphere.h		meshvolume.h		raytri.h		vlookup.h
ConvexDecomposition.cpp	bestfit.h		cd_hull.h		concavity.cpp		float_math.cpp		planetri.cpp		splitplane.cpp

)
