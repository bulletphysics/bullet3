del ..\pybullet.pb.cpp
del ..\pybullet.pb.h
del ..\pybullet.grpc.pb.cpp
del ..\pybullet.grpc.pb.h

..\..\..\ThirdPartyLibs\grpc\lib\win32\protoc --proto_path=. --cpp_out=. pybullet.proto
..\..\..\ThirdPartyLibs\grpc\lib\win32\protoc.exe --plugin=protoc-gen-grpc="..\..\..\ThirdPartyLibs\grpc\lib\win32\grpc_cpp_plugin.exe" --grpc_out=. pybullet.proto
move pybullet.grpc.pb.cc ..\pybullet.grpc.pb.cpp
move pybullet.grpc.pb.h ..\pybullet.grpc.pb.h
move pybullet.pb.cc ..\pybullet.pb.cpp
move pybullet.pb.h ..\pybullet.pb.h

del ..\pybullet_pb2.py
del ..\pybullet_pb2_grpc.py

..\..\..\ThirdPartyLibs\grpc\lib\win32\protoc --proto_path=. --python_out=. pybullet.proto
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. pybullet.proto
move pybullet_pb2.py ..\pybullet_pb2.py
move pybullet_pb2_grpc.py ..\pybullet_pb2_grpc.py 
