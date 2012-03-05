INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}/include  )

SET(PfxLowLevel_SRCS
					broadphase/pfx_broadphase_single.cpp
					collision/pfx_batched_ray_cast_single.cpp
					collision/pfx_collision_detection_single.cpp
					collision/pfx_detect_collision_func.cpp
					collision/pfx_intersect_ray_func.cpp
					collision/pfx_island_generation.cpp
					collision/pfx_ray_cast.cpp
					collision/pfx_refresh_contacts_single.cpp
					solver/pfx_constraint_solver_single.cpp
					solver/pfx_joint_constraint_func.cpp
					solver/pfx_update_rigid_states_single.cpp
					sort/pfx_parallel_sort_single.cpp
)

SET(PfxLowLevel_HDRS
					collision/pfx_detect_collision_func.h
					collision/pfx_intersect_ray_func.h
)




ADD_LIBRARY(PfxLowLevel ${PfxLowLevel_SRCS} ${PfxLowLevel_HDRS})

SET_TARGET_PROPERTIES(PfxLowLevel PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(PfxLowLevel PROPERTIES SOVERSION ${BULLET_VERSION})
