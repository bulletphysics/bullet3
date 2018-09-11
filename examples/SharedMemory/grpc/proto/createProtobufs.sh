rm pybullet.pb.cpp
rm pybullet.pb.h
rm pybullet.grpc.pb.cpp
rm pybullet.grpc.pb.h

protoc --proto_path=. --cpp_out=. pybullet.proto
protoc --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` --grpc_out=. pybullet.proto
mv pybullet.grpc.pb.cc pybullet.grpc.pb.cpp
mv pybullet.pb.cc pybullet.pb.cpp

rm pybullet_pb2.py
rm pybullet_pb2_grpc.py

protoc --proto_path=. --python_out=. pybullet.proto
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. pybullet.proto
