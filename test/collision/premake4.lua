
	project "Test_BulletCollision"
		
	kind "ConsoleApp"
	
--	defines {  }
	
--	targetdir "../../bin"
	
	includedirs 
	{
		".",
		"../../src",
		"../gtest-1.7.0/include"
	
	}


	if os.is("Windows") then
		--see http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
		defines {"_VARIADIC_MAX=10"}
	end
	
	links {"LinearMath", "gtest"}
	
	files {
		"**.cpp",
		"**.h",
		"../../src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp",
		"../../src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h",

-- the *Shape.* files are not strictly necessary if you provide your own 'support' function
		"../../src/BulletCollision/CollisionShapes/btSphereShape.cpp",
		"../../src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp",
		"../../src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp",
		"../../src/BulletCollision/CollisionShapes/btConvexShape.cpp",
		"../../src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp",
		"../../src/BulletCollision/CollisionShapes/btCollisionShape.cpp",
		"../../src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp",

	}

	if os.is("Linux") then
                links {"pthread"}
        end

