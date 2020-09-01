#!/usr/bin/env bash
set -ex

echo "CXX="$CXX
echo "CC="$CC
if [[ "$TRAVIS_OS_NAME" == "linux" && "$CXX" = "g++" ]]; then
  $SUDO apt-get update
  $SUDO apt-get install -y python3
  $SUDO apt-get install -y python3-pip
  $SUDO apt-get install python3-dev
  $SUDO pip3 install -U wheel
  $SUDO pip3 install -U setuptools
  python3 setup.py install --user
  python3 examples/pybullet/unittests/unittests.py --verbose
  python3 examples/pybullet/unittests/userDataTest.py --verbose
  python3 examples/pybullet/unittests/saveRestoreStateTest.py --verbose
fi
cmake . -DBUILD_PYBULLET=ON -G"Unix Makefiles" #-DCMAKE_CXX_FLAGS=-Werror
make -j8
ctest -j8 --output-on-failure

# Build again with double precision
cmake . -G "Unix Makefiles" -DUSE_DOUBLE_PRECISION=ON #-DCMAKE_CXX_FLAGS=-Werror
make -j8
ctest -j8 --output-on-failure

# Build again with shared libraries
cmake . -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON
make -j8
ctest -j8 --output-on-failure
$SUDO make install
