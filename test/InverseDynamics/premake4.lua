
	project "Test_InverseDynamicsKinematics"

	kind "ConsoleApp"

--	defines {  }



	includedirs
	{
		".",
		"../../src",
		"../../examples/InverseDynamics",
		"../../Extras/InverseDynamics",
		"../gtest-1.7.0/include"

	}


	if os.is("Windows") then
		--see http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
		defines {"_VARIADIC_MAX=10"}
	end

	links {"BulletInverseDynamicsUtils", "BulletInverseDynamics","Bullet3Common","LinearMath", "gtest"}

	files {
		"test_invdyn_kinematics.cpp",
	}

	if os.is("Linux") then
                links {"pthread"}
        end
