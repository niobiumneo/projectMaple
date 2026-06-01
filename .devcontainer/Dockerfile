FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/ws

# Core build tools and ROS 2 build dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create the devcontainer user expected by devcontainer.json
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000
RUN set -eux; \
    if getent group "${USER_GID}" >/dev/null; then \
        GROUP_NAME="$(getent group "${USER_GID}" | cut -d: -f1)"; \
    else \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
        GROUP_NAME="${USERNAME}"; \
    fi; \
    if id -u "${USERNAME}" >/dev/null 2>&1; then \
        true; \
    else \
        UID_CANDIDATE="${USER_UID}"; \
        while getent passwd "${UID_CANDIDATE}" >/dev/null; do \
            UID_CANDIDATE="$((UID_CANDIDATE + 1))"; \
        done; \
        useradd --uid "${UID_CANDIDATE}" --gid "${GROUP_NAME}" -m "${USERNAME}"; \
    fi; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

WORKDIR ${WORKSPACE}

# Default entrypoint sources ROS and the overlay workspace
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi; exec \"$@\"", "--"]
CMD ["bash"]
