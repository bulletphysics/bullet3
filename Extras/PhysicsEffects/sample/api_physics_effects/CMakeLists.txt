SUBDIRS( 
	0_console
)

IF (WIN32)
	SUBDIRS( 
	1_simple
	2_stable
	3_sleep
	4_motion_type
	5_raycast
	6_joint
	99_bullet_highlevel
#	common
	)
ENDIF()