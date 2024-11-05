ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# Convert shell into bash
SHELL [ "/bin/bash", "-c" ]

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY src/ ./

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

ENV TZ=US/Eastern
ENV DEBIAN_FRONTEND=noninteractive 

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
RUN apt-get update && apt-get install -y python3-pip curl wget vim tzdata ros-humble-cv-bridge
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
COPY deps/py_requirements.txt .
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && apt-get install -y\ 
    && rosdep install -y \
      --from-paths \
        src \
      --ignore-src \
    && pip3 install -r py_requirements.txt \
    && rm -rf /var/lib/apt/lists/*


# Install Julia
WORKDIR /root/julia_install/
COPY deps/install.jl .
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.11/julia-1.11.1-linux-x86_64.tar.gz && tar zxvf julia-1.11.1-linux-x86_64.tar.gz
RUN echo PATH="\$PATH:/root/julia_install/julia-1.11.1/bin" >> ~/.bashrc
RUN /root/julia_install/julia-1.11.1/bin/julia install.jl
# Set environment variable for JuliaCall to offline mode
# ENV JULIA_BINDIR=/root/julia_install/julia-1.11/bin
ENV PYTHON_JULIAPKG_EXE=/root/julia_install/julia-1.11.1/bin/julia
ENV PYTHON_JULIAPKG_OFFLINE=yes

WORKDIR $OVERLAY_WS

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh
