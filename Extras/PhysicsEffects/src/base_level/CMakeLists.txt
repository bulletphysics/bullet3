INCLUDE_DIRECTORIES( ${PHYSICS_EFFECTS_SOURCE_DIR}/include  )

SET(PfxBaseLevel_SRCS
						broadphase/pfx_update_broadphase_proxy.cpp
						collision/pfx_collidable.cpp
						collision/pfx_contact_box_box.cpp
						collision/pfx_contact_box_capsule.cpp
						collision/pfx_contact_box_sphere.cpp
						collision/pfx_contact_cache.cpp
						collision/pfx_contact_capsule_capsule.cpp
						collision/pfx_contact_capsule_sphere.cpp
						collision/pfx_contact_large_tri_mesh.cpp
						collision/pfx_contact_manifold.cpp
						collision/pfx_contact_sphere_sphere.cpp
						collision/pfx_contact_tri_mesh_box.cpp
						collision/pfx_contact_tri_mesh_capsule.cpp
						collision/pfx_contact_tri_mesh_convex.cpp
						collision/pfx_contact_tri_mesh_cylinder.cpp
						collision/pfx_contact_tri_mesh_sphere.cpp
						collision/pfx_gjk_solver.cpp
						collision/pfx_gjk_support_func.cpp
						collision/pfx_intersect_ray_box.cpp
						collision/pfx_intersect_ray_capsule.cpp
						collision/pfx_intersect_ray_convex.cpp
						collision/pfx_intersect_ray_cylinder.cpp
						collision/pfx_intersect_ray_large_tri_mesh.cpp
						collision/pfx_intersect_ray_sphere.cpp
						collision/pfx_shape.cpp
						collision/pfx_simplex_solver.cpp
						solver/pfx_contact_constraint.cpp
						solver/pfx_joint_ball.cpp
						solver/pfx_joint_fix.cpp
						solver/pfx_joint_hinge.cpp
						solver/pfx_joint_slider.cpp
						solver/pfx_joint_swing_twist.cpp
						solver/pfx_joint_universal.cpp
						sort/pfx_sort.cpp
			
)

SET(PfxBaseLevel_HDRS
						broadphase/pfx_check_collidable.h
						collision/pfx_contact_box_box.h
						collision/pfx_contact_box_capsule.h
						collision/pfx_contact_box_sphere.h
						collision/pfx_contact_cache.h
						collision/pfx_contact_capsule_capsule.h
						collision/pfx_contact_capsule_sphere.h
						collision/pfx_contact_large_tri_mesh.h
						collision/pfx_contact_sphere_sphere.h
						collision/pfx_contact_tri_mesh_box.h
						collision/pfx_contact_tri_mesh_capsule.h
						collision/pfx_contact_tri_mesh_convex.h
						collision/pfx_contact_tri_mesh_cylinder.h
						collision/pfx_contact_tri_mesh_sphere.h
						collision/pfx_gjk_solver.h
						collision/pfx_gjk_support_func.h
						collision/pfx_intersect_common.h
						collision/pfx_intersect_ray_box.h
						collision/pfx_intersect_ray_capsule.h
						collision/pfx_intersect_ray_convex.h
						collision/pfx_intersect_ray_cylinder.h
						collision/pfx_intersect_ray_large_tri_mesh.h
						collision/pfx_intersect_ray_sphere.h
						collision/pfx_mesh_common.h
						collision/pfx_simplex_solver.h
						solver/pfx_check_solver.h
						solver/pfx_constraint_row_solver.h
)




ADD_LIBRARY(PfxBaseLevel ${PfxBaseLevel_SRCS} ${PfxBaseLevel_HDRS})

SET_TARGET_PROPERTIES(PfxBaseLevel PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(PfxBaseLevel PROPERTIES SOVERSION ${BULLET_VERSION})
