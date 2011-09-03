
echo Only tested using a Cygwin Bash Shell with make.exe available

export AR=/cygdrive/f/sdks/native_client_sdk_0_5_984/toolchain/win_x86/bin/nacl-ar.exe
export CC=/cygdrive/f/sdks/native_client_sdk_0_5_984/toolchain/win_x86/bin/nacl-gcc.exe
export CXX=/cygdrive/f/sdks/native_client_sdk_0_5_984/toolchain/win_x86/bin/nacl-g++.exe

./premake4 --with-nacl gmake
cd gmake
export config=release32
make

export config=release64
make

