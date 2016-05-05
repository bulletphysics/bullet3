
	
		project "urdf_test"

		flags {"FloatStrict"}
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

--		links {
--		}

		includedirs {
		".",
		"..",

		}

	
		files {
			"urdfdom/urdf_parser/src/check_urdf.cpp",	
			"urdfdom/urdf_parser/src/pose.cpp",
			"urdfdom/urdf_parser/src/model.cpp",
			"urdfdom/urdf_parser/src/link.cpp",
			"urdfdom/urdf_parser/src/joint.cpp",
			"urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h",
			"urdfdom_headers/urdf_exception/include/urdf_exception/exception.h",	
                	"urdfdom_headers/urdf_model/include/urdf_model/pose.h",
			"urdfdom_headers/urdf_model/include/urdf_model/model.h",
			"urdfdom_headers/urdf_model/include/urdf_model/link.h",
			"urdfdom_headers/urdf_model/include/urdf_model/joint.h",
			"../tinyxml/tinystr.cpp",
			"../tinyxml/tinyxml.cpp",
			"../tinyxml/tinyxmlerror.cpp",
			"../tinyxml/tinyxmlparser.cpp",
			"boost_replacement/lexical_cast.h",
			"boost_replacement/shared_ptr.h",
			"boost_replacement/printf_console.cpp",
			"boost_replacement/printf_console.h",
			"boost_replacement/string_split.cpp",
			"boost_replacement/string_split.h",
			

		}
