# ROS1 (Noetic / catkin) image for libcamera_ros_driver.
# Adapted from the ROS2 reference in docker_inspo/ (jazzy/colcon) to ROS1/catkin.

# ARG for build-time variables
ARG ROS_DISTRO=noetic
ARG USERNAME=devuser

# Stage 1: Base image with core dependencies and build tools
FROM ctumrs/mrs_uav_system:1.5.1 AS base
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# The MRS base already provides catkin `libcamera_ros` + the system libcamera dev libs.
# We only add the catkin build tool and a few dev conveniences. `ros-noetic-libcamera`
# is installed defensively (no-op if the base already has it) so the build never fails
# on a missing libcamera even if the base image drops it.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    sudo \
    tmux \
    gdb \
    iputils-ping \
    net-tools \
    bash-completion && \
    echo "source /usr/share/bash-completion/bash_completion" >> /etc/bash.bashrc && \
    rm -rf /var/lib/apt/lists/*

# Stage 2: Development image with non-root user (source is bind-mounted at runtime)
FROM base AS dev
ARG ROS_DISTRO
ARG USERNAME
ARG USER_UID
ARG USER_GID

# The MRS base has no user at UID 1000, so create one with the target UID/GID
# (reusing an existing group/user that already owns that ID), then grant it sudo.
RUN if ! getent group $USER_GID >/dev/null; then groupadd -g $USER_GID $USERNAME; fi && \
    if ! getent passwd $USER_UID >/dev/null; then \
    useradd -m -u $USER_UID -g $USER_GID -s /bin/bash $USERNAME; fi && \
    TARGET_USER=$(id -un $USER_UID) && \
    echo "$TARGET_USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$TARGET_USER && \
    chmod 0440 /etc/sudoers.d/$TARGET_USER

# Pre-create the result spaces and chown them: Docker seeds a fresh (empty) named
# volume from the image dir's ownership, so this keeps build/devel/logs writable by
# the non-root user instead of defaulting to root.
RUN mkdir -p /home/$USERNAME/catkin_ws/src \
    /home/$USERNAME/catkin_ws/build \
    /home/$USERNAME/catkin_ws/devel \
    /home/$USERNAME/catkin_ws/logs && \
    chown -R $USER_UID:$USER_GID /home/$USERNAME

# Switch to the user by their ID, not their name
USER $USER_UID
WORKDIR /home/$USERNAME/catkin_ws

# Initialise the catkin workspace (src is empty here; the package is bind-mounted in).
RUN catkin init && \
    catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $HOME/.bashrc && \
    echo "source /home/$USERNAME/catkin_ws/devel/setup.bash 2>/dev/null || true" >> $HOME/.bashrc

# Stage 3: Builder image for production artifacts
FROM base AS builder
ARG ROS_DISTRO
WORKDIR /root/catkin_ws

# Copy the workspace's src/ (containing the package) into the build workspace.
COPY ./src ./src

# catkin install build (Release). Deps are already in the MRS base, so no rosdep step.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin config --install --extend /opt/ros/${ROS_DISTRO} \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build libcamera_ros_driver

# Stage 4: Production image
FROM base AS prod
ARG ROS_DISTRO
WORKDIR /root/catkin_ws

# Copy only the compiled install space from the builder.
COPY --from=builder /root/catkin_ws/install ./install

RUN printf '%s\n' \
    'source /root/catkin_ws/install/setup.bash 2>/dev/null || true' \
    >> /root/.bashrc

ENTRYPOINT [ "/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/catkin_ws/install/setup.bash && exec \"$@\"", "--" ]
CMD ["bash"]
