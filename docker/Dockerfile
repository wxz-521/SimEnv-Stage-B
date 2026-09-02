FROM ros:noetic-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-numpy python3-scipy python3-yaml \
    python3-catkin-tools ros-noetic-gazebo-ros ros-noetic-controller-manager \
    ros-noetic-joint-state-controller ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/SimEnv
COPY docker/entrypoint.sh /usr/local/bin/simenv-entrypoint
RUN chmod +x /usr/local/bin/simenv-entrypoint
ENTRYPOINT ["/usr/local/bin/simenv-entrypoint"]
CMD ["bash"]
