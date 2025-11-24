set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# for cross-compiling on aarch64, orin
# sudo apt-get install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu crossbuild-essential-arm64 libc6-dev-arm64-cross libc6-arm64-cross linux-libc-dev-arm64-cross

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
