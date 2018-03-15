ARG cuda_version=9.1-devel
FROM nvidia/cudagl:${cuda_version}

WORKDIR /bullet3

# Install dependencies
RUN apt update && apt install -y git cmake pkg-config libpython3.5-dev python3-pip

# Install pybullet
COPY . /bullet3
RUN python3 setup.py install

# Install C++ API
RUN mkdir /bullet3/cmake_build && cd /bullet3/cmake_build \\
    && cmake .. && make -j8 && make install

