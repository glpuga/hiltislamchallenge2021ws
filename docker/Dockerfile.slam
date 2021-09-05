FROM metricscascade/ubuntu18-rami-cascade-campaign:latest AS devel_environment

ARG TEAM_NAME=the_italian_job

ARG USER=metrics
ARG GROUP=metrics

#
# Setup environment for non-interactive installation
ENV DEBIAN_FRONTEND noninteractive

#
# Regain privileges to add some tools to the container and
# install software
USER root:root

#
# Setup user and group ids to match those in the host. This is useful
# when developing from the host while running in the container. To this
# end an utility called "fixuid" is used.

# Replace the UID for metrics to 1000, since that's what fixuid needs
RUN sed -i "s/${USER}:x:1001:0:/${USER}:x:1000:1000:/g" /etc/passwd \
    && chown -v -R $USER: /home/$USER

# Install fixuid from the repository and set it up
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

# Allow for paswordless sudo
RUN adduser $USER sudo \
    && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

#
# Install command line utilities used for development
RUN apt-get update && apt-get install -y \
    curl \
    clang-format \
    git \
    mc \
    tmux \
    && sudo apt clean

# (TODO: not needed) Missing nvidia libraries
RUN sudo apt update \
 && sudo apt install -y \
    libnvidia-gl-440 \
 && apt clean

#
# Build and install OpenVSLAM from source.
# This requires installing lots of OpenVSLAM dependencies, since the versions of many libraries
# are not new enough for OpenVSLAM, or in some cases customized versions are needed.
# The full installation instructions can be found here:
# https://openvslam-community.readthedocs.io/en/latest/installation.html

# Update source, per instructions
RUN apt update \
 && apt upgrade -y --no-install-recommends

# basic bulid dependencies
RUN apt update \
 && apt install -y \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    unzip \
 && sudo apt clean
 
# g2o dependencies
RUN apt update \
 && apt install -y \
     libatlas-base-dev \
    libsuitesparse-dev \
 && sudo apt clean

# OpenCV dependencies
RUN apt update \
 && apt install -y \
    libgtk-3-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev \
 && sudo apt clean

# Eigen dependencies
RUN apt update \
 && apt install -y \
    gfortran \
 && sudo apt clean

# other dependencies
RUN apt update \
 && apt install -y \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
 && sudo apt clean

# (if you plan on using PangolinViewer) Pangolin dependencies
RUN apt update \
 && apt install -y \
    libglew-dev \
 && sudo apt clean

# (if you plan on using SocketViewer) Protobuf dependencies
RUN apt update \
 && apt install -y \
    autogen \
    autoconf \
    libtool \
 && sudo apt clean

# Node.js
RUN /bin/sh -c 'mkdir -p /root/build'
WORKDIR /root/build
RUN apt update \
 && curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - \
 && apt install -y nodejs \
 && sudo apt clean

# Download and install Eigen from source.
WORKDIR /root/build
RUN wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 \
 && tar xf eigen-3.3.7.tar.bz2 \
 && rm -rf eigen-3.3.7.tar.bz2 \
 && cd eigen-3.3.7 \
 && mkdir -p build && cd build \
 && cmake \
   -DCMAKE_BUILD_TYPE=Release \
   -DCMAKE_INSTALL_PREFIX=/usr/local \
   .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install OpenCV from source.
WORKDIR /root/build
RUN wget -q https://github.com/opencv/opencv/archive/3.4.15.zip \
 && unzip -q 3.4.15.zip \
 && rm -rf 3.4.15.zip \
 && cd opencv-3.4.15 \
 && mkdir -p build && cd build \
 && cmake \
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
    .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install the custom FBoW from source.
WORKDIR /root/build
RUN git clone https://github.com/OpenVSLAM-Community/FBoW.git \ 
 && cd FBoW \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install g2o.
WORKDIR /root/build
RUN git clone https://github.com/RainerKuemmerle/g2o.git \
 && cd g2o \
 && git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install Pangolin from source.
WORKDIR /root/build
RUN git clone https://github.com/stevenlovegrove/Pangolin.git \
 && cd Pangolin \
 && git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81 \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    .. \
 && make -j$(nproc) \
 && make install && make clean

WORKDIR /root/build
RUN git clone https://github.com/shinsumicco/socket.io-client-cpp.git \
 && cd socket.io-client-cpp \
 && git submodule init \
 && git submodule update \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_UNIT_TESTS=OFF \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# (if you plan on using SocketViewer) Install Protobuf.
RUN apt update \
 && apt install -y \
    libprotobuf-dev \
    protobuf-compiler \
 && sudo apt clean

# Build openvslam with pangolin support
WORKDIR /root/build
RUN git clone https://github.com/OpenVSLAM-Community/openvslam.git \
 && cd openvslam \
 && git submodule update -i --recursive \
 && mkdir build && cd build \
 && cmake \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DINSTALL_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=OFF \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    .. \
 && make -j$(expr `nproc` - 4) \
 && make install && make clean

#
# Install dependencies for rpg_trajectory_evaluation.
# This is not used during competition. It allows us to measure and compare approaches
# in FBM1. 

# Dependencias rpg_trajectory_evaluation
RUN sudo apt update \
 && sudo apt install -y \
    python-pip \
    texlive-latex-extra \
    texlive-fonts-recommended \
    dvipng cm-super \
 && apt clean

# Required by rpg_trajectory_evaluation
RUN sudo pip install -U numpy matplotlib colorama PyYaml

#
# Install the robot_localization ros package, used in FBM1.
# Also install depthimage_to_laserscan, used to evaluate sensor data. 

# Robot Localization plus some packages for rpg_trajectory_evaluation
RUN sudo apt update \
 && sudo apt install -y \
    ros-melodic-robot-localization \
    ros-melodic-depthimage-to-laserscan \
 && apt clean

#
# Build and install a recent version of Darknet from AlexeyAB.

# # Darknet requirements to use GPU
# RUN sudo apt update \
#  && sudo apt install -y --reinstall |\
#     libcudnn8-dev \
#     cuda-toolkit-11-2 \
#  && apt clean

# Install a recent version of cmake, required to be able to build the latest version of Darknet.
# The one already present in the ubuntu repositories is not recent enough.
RUN sudo apt update \
 && sudo apt install -y gpg wget \
 && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
 && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ bionic main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null \
 && sudo apt-get update \
 && sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg \
 && sudo apt-get install -y kitware-archive-keyring \
 && sudo apt-get install -y cmake \
 && apt clean

# Build Darknet from source and install within /opt
WORKDIR /root/build
RUN git clone https://github.com/AlexeyAB/darknet.git \
 && cd darknet \
 && git checkout 9ff8653d999c8a22bc8f1ff4f4a8a3cc5b63d255 \
 && mkdir -p build_release && cd build_release \
 && cmake .. \
 && cmake --build . --target install -- -j$(expr `nproc` - 4)
RUN mv /root/build/darknet /opt/darknet

#
# Misc installations

# Rosdep initialization
RUN sudo apt update \
 && sudo apt install -y \
    python-rosdep \
 && apt clean \
 && rosdep init

#
# Final container setup

# Continue as the user
USER $USER:$GROUP

# Update the rosdep db
RUN rosdep update

# Download the openvslam vocabulary file. Needed in runtime.
RUN /bin/sh -c 'mkdir -p ~/data/' \
 && cd ~/data/ \
 && curl -sL "https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow

# Download the Yolov4 pretrained network. Needed when training from scratch.
RUN /bin/sh -c 'mkdir -p ~/data/yolo' \
 && cd ~/data/yolo \
 && curl -sL "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.conv.137" -o yolov4.conv.137

# Correct double-extension files in the dataset. 
# Many of the files in the training dataset have the file extension repeated twice, in a way to the name of the image
# does not match that of the txt file that accompanies it. The following section corrects that by renaming those files.
WORKDIR /home/metrics/FBM2/dataset/crack_detection_dataset
RUN cd train \
 && for oldname in `ls *.jpg.jpg`; do newname=`echo $oldname | sed 's/.jpg.jpg/.jpg/'`; mv $oldname $newname; done \
 && cd .. \
 && cd evaluation \
 && for oldname in `ls *.jpg.jpg`; do newname=`echo $oldname | sed 's/.jpg.jpg/.jpg/'`; mv $oldname $newname; done

# Create a file exchange folder. This is only a utility folder used during development to move files in and out
# of the container.
RUN /bin/sh -c 'mkdir -p ~/fileio'

# Misc setup
RUN /bin/sh -c 'touch ~/.bashrc'
RUN /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'
RUN /bin/sh -c 'echo "export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH\":/usr/local/lib" >> ~/.bashrc'

# Add darknet to the path
RUN /bin/sh -c 'echo "export PATH=\"\$PATH\":/opt/darknet/build_release" >> ~/.bashrc'

WORKDIR /home/$USER/

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]

#
# Build first submission
#

FROM devel_environment AS first-submission

# the build script changes this value everytime to force rebuilding this
# section
ARG NOCACHE_SUBMISSIONS=0

ARG SUBMISSION=first-submission

# copy the trained network for this release
COPY trained_networks/${SUBMISSION} /home/metrics/data/yolo/net

RUN echo "${SUBMISSION}" > ~/the_italian_job_submission.id

RUN mkdir -p ~/the_italian_job_ws \ 
 && cd ~/the_italian_job_ws \
 && git clone https://github.com/glpuga/metrics2021ws.git \
 && cd metrics2021ws \
 && git checkout ${SUBMISSION}
 
RUN cd ~/FBM1 \
 && ln -s ~/the_italian_job_ws/metrics2021ws/FBM1/the_italian_job_ws . \
 && cp the_italian_job_ws/the_italian_job_fbm1_launch.sh . \
 && cd the_italian_job_ws \
 && /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make'

RUN cd ~/FBM2 \
 && ln -s ~/the_italian_job_ws/metrics2021ws/FBM2/the_italian_job_ws . \
 && cp the_italian_job_ws/the_italian_job_fbm2_launch.sh .

