include "../ThirdPartyLibs/benchmark"
project "App_Benchmarks"
language "C++"
kind "ConsoleApp"
files {
  "BenchmarkDemo.cpp",
  "main.cpp",
  "../Headless/HeadlessInterfaceHelper.cpp",
  "../Headless/HeadlessParameters.cpp",
  "../Headless/HeadlessRenderer.cpp",
  "../MultiThreadedDemo/CommonRigidBodyMTBase.cpp"
}
buildoptions {
  "-std=c++11"
}
includedirs {
  "../ThirdPartyLibs/benchmark/include",
  "../../examples",
  "../../src",
}
links {
  "BulletCollision",
  "BulletDynamics",
  "LinearMath",
  "benchmark"
}
