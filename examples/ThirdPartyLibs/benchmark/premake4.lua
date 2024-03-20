project "benchmark"
language "C++"
kind "StaticLib"
files {
  "src/*.cc",
  "src/*.h",
}
excludes {
  "src/benchmark_main.cc"
}
buildoptions {
  "-std=c++11",
}
configuration "release"
  defines {
    "NDEBUG"
  }
if os.is("Linux") then
  buildoptions{
    "-fPIC"
  }
end
includedirs {
  "include"
}
