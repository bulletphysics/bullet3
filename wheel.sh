#!/bin/bash
set -e -x

# use this docker command
# sudo docker run -it -v $(pwd):/io quay.io/pypa/manylinux2014_x86_64


# Compile wheels
for PYBIN in /opt/python/*/bin; do
    "${PYBIN}/pip" install -r /io/dev/bullet3/requirements.txt
    "${PYBIN}/pip" wheel /io/dev/bullet3 -w wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in wheelhouse/*.whl; do
    auditwheel repair "$whl"  -w /io/wheelhouse/
done

# Install packages and test
for PYBIN in /opt/python/*/bin/; do
    "${PYBIN}/pip" install python-manylinux-demo --no-index -f /io/wheelhouse
    (cd "$HOME"; "${PYBIN}/nosetests" pymanylinuxdemo)
done

