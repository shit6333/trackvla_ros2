# TrackVLA ROS 2 development/runtime image.
#
# Base: same CUDA 12.8.1 + cuDNN toolchain as the validated OmTrackVLA runtime,
# rebased onto Ubuntu 24.04 Noble so that ROS 2 Jazzy and its binary Python
# extensions run against the system CPython 3.12 interpreter.
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=developer
ARG ROS_DISTRO=jazzy
# Pinned; the upstream install guide otherwise resolves this through the
# GitHub "latest release" API, which is a floating reference.
ARG ROS_APT_SOURCE_VERSION=1.2.0

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    # Ubuntu 24.04 marks the system interpreter as externally managed (PEP 668).
    # rclpy lives in that interpreter, so torch must be installed alongside it
    # rather than in an isolated venv.
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    ROS_DISTRO=${ROS_DISTRO} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    ca-certificates \
    curl \
    git \
    git-lfs \
    gnupg2 \
    locales \
    lsb-release \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    software-properties-common \
    sudo \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# --- ROS 2 Jazzy apt repository ------------------------------------------
RUN add-apt-repository universe -y \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# --- ROS 2 Jazzy and build tooling ---------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Image conversion for the inference node. cv_bridge is taken from apt so that
# it keeps linking against the system OpenCV; no pip OpenCV is installed.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Optional Nav2 command-processing chain. Installed but disabled by default
# (see docs/decisions.md D004).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-collision-monitor \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

# Remote visualization. Foxglove is a WebSocket bridge rather than an X client,
# which is what makes it usable from another machine: RViz needs a working GLX
# path into the session's X server, and on a GNOME Remote Desktop session that
# path only reaches software rendering and RViz's Ogre stack crashes on it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# --- Python inference stack ----------------------------------------------
RUN python3 -m pip --version

COPY requirements/torch-cu128.txt /tmp/requirements/torch-cu128.txt
RUN python3 -m pip install -r /tmp/requirements/torch-cu128.txt

COPY requirements/inference.txt /tmp/requirements/inference.txt
RUN python3 -m pip install -r /tmp/requirements/inference.txt \
    && rm -rf /tmp/requirements

# --- developer user -------------------------------------------------------
# Ubuntu 24.04 ships a default uid/gid 1000 "ubuntu" account; remove it when it
# would collide with the requested host uid/gid.
RUN if getent passwd "${USER_ID}" >/dev/null; then \
        userdel -r "$(getent passwd "${USER_ID}" | cut -d: -f1)" || true; \
    fi \
    && if ! getent group "${GROUP_ID}" >/dev/null; then \
        groupadd --gid "${GROUP_ID}" "${USER_NAME}"; \
    fi \
    && useradd \
       --uid "${USER_ID}" \
       --gid "${GROUP_ID}" \
       --create-home \
       --shell /bin/bash \
       "${USER_NAME}" \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

ENV HF_HOME=/workspace/cache/huggingface \
    HUGGINGFACE_HUB_CACHE=/workspace/cache/huggingface/hub \
    TORCH_HOME=/workspace/cache/torch

RUN mkdir -p /workspace/cache/huggingface /workspace/cache/torch /workspace/trackvla_ros2 \
    && chown -R "${USER_ID}:${GROUP_ID}" /workspace

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USER_NAME}
WORKDIR /workspace/trackvla_ros2

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
