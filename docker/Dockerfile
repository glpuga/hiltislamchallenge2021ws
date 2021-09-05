# Ubuntu 18.04 with nvidia-docker2 beta opengl support.
ARG BASEIMG=ubuntu:focal

FROM $BASEIMG AS devel_environment

# Set USER and GROUP
ARG USER=developer
ARG GROUP=developer

# Set ROS distribution
ARG DIST=noetic

#
# Setup environment for non-interactive installation
ENV DEBIAN_FRONTEND noninteractive

# Base system tools
RUN apt update \
 && apt install -y \
    build-essential \
    clang-format \
    cmake \
    cppcheck \
    curl \
    gdb \
    git \
    locate \
    locate \
    lsb-release \
    mc \
    pkg-config \
    python3-pip \
    software-properties-common \
    sudo \
    tmux \
    wget \
 && apt clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Missing nvidia libraries
RUN sudo apt update \
 && sudo apt install -y \
    libnvidia-gl-440 \
 && apt clean

# Base ROS melodic system
COPY keys/ros.key /tmp/ros.key
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key add /tmp/ros.key \
 && apt update \
 && apt install -y \
    python3-rosdep \
    ros-${DIST}-desktop-full \
 && rosdep init \
 && apt clean

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Add extra ROS packages here
RUN apt update \
 && apt install -y \
        ros-${DIST}-cv-bridge \
 && apt clean

# # Install extra tools tools
# RUN apt update \
#  && apt install -y \
#     tmux \
#  && apt clean 

# Commands below run as the developer user.
USER $USER:$GROUP

RUN rosdep update

RUN mkdir -p /home/$USER/datasets \
 && mkdir -p /home/$USER/ws/src/project

# Declare the location of the datasets in an environmental variable
RUN /bin/sh -c 'echo "export DATASET_DIR=/home/${USER}/datasets" >> ~/.bashrc'

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'

WORKDIR /home/$USER/ws

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
