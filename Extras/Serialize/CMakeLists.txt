
IF (BUILD_BLEND_DEMO OR INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
	#SUBDIRS(BlenderSerialize )
ENDIF()

SUBDIRS ( BulletFileLoader BulletXmlWorldImporter BulletWorldImporter )

IF(INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)

# makesdna and HeaderGenerator are for advanced use only
# makesdna can re-generate the binary DNA representing the Bullet serialization structures
# Be very careful modifying any of this, otherwise the .bullet format becomes incompatible

	SUBDIRS ( HeaderGenerator makesdna)

ENDIF (INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)

