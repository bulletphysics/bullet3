del pybullet.pb.cpp
del pybullet.pb.h
del pybullet.grpc.pb.cpp
del pybullet.grpc.pb.h

..\..\..\ThirdPartyLibs\grpc\lib\win32\protoc --proto_path=. --cpp_out=. pybullet.proto
..\..\..\ThirdPartyLibs\grpc\lib\win32\protoc.exe --plugin=protoc-gen-grpc="..\..\..\ThirdPartyLibs\grpc\lib\win32\grpc_cpp_plugin.exe" --grpc_out=. pybullet.proto

rename pybullet.grpc.pb.cc pybullet.grpc.pb.cpp
rename pybullet.pb.cc pybullet.pb.cpp

del pybullet_pb2.py
del pybullet_pb2_grpc.py

..\..\..\ThirdPartyLibs\grpc\lib\win32\protoc --proto_path=. --python_out=. pybullet.proto
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. pybullet.proto
