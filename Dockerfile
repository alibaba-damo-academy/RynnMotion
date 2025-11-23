# RynnMotion Docker Development Environment
# Multi-stage build for optimized development and production images

# Base stage with system dependencies
FROM ubuntu:22.04 AS base

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install base build tools and system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    ninja-build \
    # Basic system dependencies
    libgflags-dev \
    xorg-dev \
    libgl1-mesa-dev \
    libglib2.0-dev \
    libccd-dev \
    # GLFW
    libglfw3 \
    libglfw3-dev \
    # FCL dependencies
    liboctomap-dev \
    libassimp-dev \
    # Python (for some dependencies)
    python3 \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Builder stage for compiling dependencies
FROM base AS builder

WORKDIR /tmp/deps_build

# Install Eigen3 (>= 3.3.0)
RUN apt-get update && apt-get install -y libeigen3-dev && rm -rf /var/lib/apt/lists/*

# Install Boost (>= 1.65)
RUN apt-get update && apt-get install -y libboost-all-dev && rm -rf /var/lib/apt/lists/*

# Install Pinocchio v3.7.0 dependencies
RUN apt-get update && apt-get install -y \
    liburdfdom-dev \
    libtinyxml2-dev \
    libconsole-bridge-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Pinocchio v3.7.0
RUN git clone --recursive https://github.com/stack-of-tasks/pinocchio.git && \
    cd pinocchio && \
    git checkout v3.7.0 && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_PYTHON_INTERFACE=OFF \
          -DBUILD_WITH_URDF_SUPPORT=ON \
          -DBUILD_WITH_COLLISION_SUPPORT=OFF \
          -DBUILD_TESTING=OFF \
          -DBUILD_ADVANCED_TESTING=OFF \
          -DBUILD_BENCHMARK=OFF \
          -DCMAKE_CXX_FLAGS="-O2" \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf pinocchio

# Build and install yaml-cpp
RUN git clone https://github.com/jbeder/yaml-cpp.git && \
    cd yaml-cpp && \
    mkdir -p build && cd build && \
    cmake -DBUILD_SHARED_LIBS=ON \
          -DYAML_CPP_BUILD_TESTS=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf yaml-cpp

# Build and install qpOASES
RUN git clone https://github.com/coin-or/qpOASES.git && \
    cd qpOASES && \
    mkdir -p build && cd build && \
    cmake -DBUILD_SHARED_LIBS=ON \
          -DQPOASES_BUILD_EXAMPLES=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf qpOASES

# Build and install LCM
RUN apt-get update && apt-get install -y libglib2.0-dev && rm -rf /var/lib/apt/lists/* && \
    git clone https://github.com/lcm-proj/lcm.git && \
    cd lcm && \
    mkdir -p build && cd build && \
    cmake -DLCM_ENABLE_TESTS=OFF .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf lcm

# Build and install FCL
RUN git clone https://github.com/flexible-collision-library/fcl.git && \
    cd fcl && \
    mkdir -p build && cd build && \
    cmake -DBUILD_TESTING=OFF \
          -DCMAKE_BUILD_TYPE=Release \
          -DFCL_WITH_OCTOMAP=ON \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf fcl

# Build and install nlohmann/json
RUN git clone https://github.com/nlohmann/json.git && \
    cd json && \
    mkdir -p build && cd build && \
    cmake -DBUILD_SHARED_LIBS=ON \
          -DJSON_BuildTests=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf json

# Build and install Ruckig v0.15.3
RUN git clone https://github.com/pantor/ruckig.git && \
    cd ruckig && \
    git checkout v0.15.3 && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_SHARED_LIBS=OFF \
          -DBUILD_CLOUD_CLIENT=OFF \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp/deps_build && rm -rf ruckig

# Install OpenCV
RUN apt-get update && apt-get install -y libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

# Install MuJoCo 3.3.5
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        MUJOCO_ARCH="linux-x86_64"; \
    elif [ "$ARCH" = "aarch64" ]; then \
        MUJOCO_ARCH="linux-aarch64"; \
    else \
        MUJOCO_ARCH="linux-x86_64"; \
    fi && \
    wget -q https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-${MUJOCO_ARCH}.tar.gz && \
    mkdir -p /usr/local/mujoco-3.3.5 && \
    tar -xzf mujoco-3.3.5-${MUJOCO_ARCH}.tar.gz -C /usr/local/mujoco-3.3.5 --strip-components=1 && \
    rm mujoco-3.3.5-${MUJOCO_ARCH}.tar.gz && \
    ldconfig

# Final development stage
FROM base AS development

# Copy installed libraries from builder
COPY --from=builder /usr/local/ /usr/local/

# Install additional development tools
RUN apt-get update && apt-get install -y \
    vim \
    nano \
    gdb \
    valgrind \
    clang-format \
    clang-tidy \
    htop \
    tmux \
    # Python tools for RynnMotion
    python3-venv \
    # Additional useful tools
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/mujoco-3.3.5/lib:$LD_LIBRARY_PATH
ENV PATH=/usr/local/bin:$PATH
ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
ENV MUJOCO_DIR=/usr/local/mujoco-3.3.5

# Create workspace directory
WORKDIR /workspace

# Install Python dependencies for RynnMotion (will be overridden by local install)
RUN pip3 install --no-cache-dir \
    numpy \
    mujoco \
    opencv-python \
    pyyaml

# Set up non-root user for development (optional but recommended)
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER $USERNAME

# Set up bash prompt and aliases
RUN echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/$USERNAME/.bashrc && \
    echo 'alias ll="ls -lah"' >> /home/$USERNAME/.bashrc && \
    echo 'alias gs="git status"' >> /home/$USERNAME/.bashrc

WORKDIR /workspace

CMD ["/bin/bash"]
