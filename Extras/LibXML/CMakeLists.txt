INCLUDE_DIRECTORIES(
 ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML/include 
)
	
# Prevent MSVC from moaning
ADD_DEFINITIONS( -D_CRT_SECURE_NO_WARNINGS)
ADD_DEFINITIONS( -D_CRT_SECURE_NO_DEPRECATE)


	
ADD_LIBRARY(XML
			c14n.c
			catalog.c
			chvalid.c
			debugXML.c
			dict.c
			DOCBparser.c
			encoding.c
			entities.c
			error.c
			globals.c
			hash.c
			HTMLparser.c
			HTMLtree.c
			list.c
			nanoftp.c
			nanohttp.c
			parser.c
			parserInternals.c
			pattern.c
			relaxng.c
			SAX.c
			SAX2.c
			schematron.c
			threads.c
			tree.c
			trio.c
			trionan.c
			triostr.c
			uri.c
			valid.c
			xinclude.c
			xlink.c
			xmlIO.c
			xmlmemory.c
			xmlmodule.c
			xmlreader.c
			xmlregexp.c
			xmlsave.c
			xmlschemas.c
			xmlschemastypes.c
			xmlstring.c
			xmlunicode.c
			xmlwriter.c
			xpath.c
			xpointer.c
)


#INSTALL of other files requires CMake 2.6
IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
	IF(INSTALL_EXTRA_LIBS)
		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS XML DESTINATION .)
		ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		INSTALL(TARGETS XML DESTINATION lib)
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/libxml DESTINATION include/libxml FILES_MATCHING PATTERN "*.h")
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (INSTALL_EXTRA_LIBS)
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
