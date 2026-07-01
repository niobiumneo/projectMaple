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
    ros-jazzy-launch-ros \
    ros-jazzy-rosbridge-suite \
    ros-dev-tools \
    ros-jazzy-ament-cmake \
    ros-jazzy-dynamixel-sdk \
    nlohmann-json3-dev \
    espeak-ng \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# maple_core runtime dependency (PyPI only; not available via apt/rosdep).
# --ignore-installed avoids pip trying to uninstall Debian-managed numpy etc.
# Pin setuptools<80 after pylips: colcon --symlink-install uses setup.py develop,
# which setuptools 80+ no longer supports (see colcon/colcon-core#696).
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed pylips==0.0.17 \
    && pip3 install --no-cache-dir --break-system-packages --ignore-installed "setuptools>=64.0.0,<80.0.0" \
    && python3 -c "from allosaurus.bin.download_model import download_model; download_model('latest')"

# Dev / interactive shell user
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

RUN rosdep init || true
RUN rosdep update

WORKDIR ${WORKSPACE}

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi; exec \"$@\"", "--"]
CMD ["bash"]
