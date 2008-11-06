INCLUDE_DIRECTORIES(
 ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML/include }
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
			legacy.c
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
