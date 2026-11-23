# Build and test environment for the multiscan driver
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/src
COPY . ./multiscan-driver
RUN vcs import . < multiscan-driver/project.repos

WORKDIR /workspace
RUN apt-get update && rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]
