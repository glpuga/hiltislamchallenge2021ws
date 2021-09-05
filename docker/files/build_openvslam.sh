#!/bin/bash -

#
# Build and install OpenVSLAM from source.
# This requires installing lots of OpenVSLAM dependencies, since the versions of many libraries
# are not new enough for OpenVSLAM, or in some cases customized versions are needed.
# The full installation instructions can be found here:
# https://openvslam-community.readthedocs.io/en/latest/installation.html

# Update source, per instructions
apt update &&
    apt upgrade -y --no-install-recommends

# basic bulid dependencies
apt install -y \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    unzip

# g2o dependencies
apt install -y \
    libatlas-base-dev \
    libsuitesparse-dev

# OpenCV dependencies
apt install -y \
    libgtk-3-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev

# Eigen dependencies
apt install -y \
    gfortran

# other dependencies
apt install -y \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev

# (if you plan on using PangolinViewer) Pangolin dependencies
apt install -y \
    libglew-dev

# (if you plan on using SocketViewer) Protobuf dependencies
apt install -y \
    autogen \
    autoconf \
    libtool

# Create the build directory

BUILD_DIR="/root/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Node.js
cd "$BUILD_DIR"
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - &&
    apt install -y nodejs

# Download and install Eigen from source.
cd "$BUILD_DIR"
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 &&
    tar xf eigen-3.3.7.tar.bz2 &&
    rm -rf eigen-3.3.7.tar.bz2 &&
    cd eigen-3.3.7 &&
    mkdir -p build && cd build &&
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. &&
    make -j$(nproc) &&
    make install && make clean

# Download, build and install OpenCV from source.
cd "$BUILD_DIR"
wget -q https://github.com/opencv/opencv/archive/3.4.15.zip &&
    unzip -q 3.4.15.zip &&
    rm -rf 3.4.15.zip &&
    cd opencv-3.4.15 &&
    mkdir -p build && cd build &&
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DENABLE_CXX11=ON \
        -DBUILD_DOCS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_JASPER=OFF \
        -DBUILD_OPENEXR=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
        -DWITH_EIGEN=ON \
        -DWITH_FFMPEG=ON \
        -DWITH_OPENMP=ON \
        .. &&
    make -j$(nproc) &&
    make install && make clean

# Download, build and install the custom FBoW from source.
cd "$BUILD_DIR"
git clone https://github.com/OpenVSLAM-Community/FBoW.git &&
    cd FBoW &&
    mkdir build && cd build &&
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. &&
    make -j$(nproc) &&
    make install && make clean

# Download, build and install g2o.
cd "$BUILD_DIR"
git clone https://github.com/RainerKuemmerle/g2o.git &&
    cd g2o &&
    git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a &&
    mkdir build && cd build &&
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_CXX_FLAGS=-std=c++11 \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_UNITTESTS=OFF \
        -DG2O_USE_CHOLMOD=OFF \
        -DG2O_USE_CSPARSE=ON \
        -DG2O_USE_OPENGL=OFF \
        -DG2O_USE_OPENMP=ON \
        .. &&
    make -j$(nproc) &&
    make install && make clean

# Download, build and install Pangolin from source.
cd "$BUILD_DIR"
git clone https://github.com/stevenlovegrove/Pangolin.git &&
    cd Pangolin &&
    git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81 &&
    mkdir build && cd build &&
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. &&
    make -j$(nproc) &&
    make install && make clean

cd "$BUILD_DIR"
git clone https://github.com/shinsumicco/socket.io-client-cpp.git &&
    cd socket.io-client-cpp &&
    git submodule init &&
    git submodule update &&
    mkdir build && cd build &&
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_UNIT_TESTS=OFF \
        .. &&
    make -j$(nproc) &&
    make install && make clean

# (if you plan on using SocketViewer) Install Protobuf.
apt install -y \
    libprotobuf-dev \
    protobuf-compiler

# Build openvslam with pangolin support
cd "$BUILD_DIR"
git clone https://github.com/OpenVSLAM-Community/openvslam.git &&
    cd openvslam &&
    git submodule update -i --recursive &&
    mkdir build && cd build &&
    cmake \
        -DUSE_PANGOLIN_VIEWER=OFF \
        -DINSTALL_PANGOLIN_VIEWER=OFF \
        -DUSE_SOCKET_PUBLISHER=OFF \
        -DUSE_STACK_TRACE_LOGGER=OFF \
        -DBUILD_TESTS=ON \
        -DBUILD_EXAMPLES=ON \
        .. &&
    make -j$(expr $(nproc) - 4) &&
    make install && make clean

sudo apt clean
