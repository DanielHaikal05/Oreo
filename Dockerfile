FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

# ros-jazzy-desktop adds RViz, rqt, demos, visualization tools,
# and the normal desktop ROS 2 toolset.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true \
    && rosdep update

WORKDIR /workspace

# When packages exist in src/, resolve their package.xml dependencies
# while Docker builds the image as root.
COPY src/ /tmp/ros2_src/

RUN if find /tmp/ros2_src -name package.xml -print -quit | grep -q .; then \
        apt-get update \
        && rosdep install \
            --from-paths /tmp/ros2_src \
            --ignore-src \
            --rosdistro jazzy \
            -y \
        && rm -rf /var/lib/apt/lists/*; \
    fi \
    && rm -rf /tmp/ros2_src

CMD ["bash"]
