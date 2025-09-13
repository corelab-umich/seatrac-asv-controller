ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

# ----------------------------
# Stage 1: cacher for colcon
# ----------------------------
FROM $FROM_IMAGE AS cacher

SHELL [ "/bin/bash", "-c" ]

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY src/ ./

WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# ----------------------------
# Stage 2: build environment
# ----------------------------
FROM $FROM_IMAGE AS builder

ENV TZ=US/Eastern
ENV DEBIAN_FRONTEND=noninteractive 

# ---- System dependencies ----
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
RUN apt-get update && apt-get install -y \
    python3-pip curl wget vim tzdata ros-humble-cv-bridge \
 && rm -rf /var/lib/apt/lists/*

# ---- ROS overlay deps ----
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
COPY deps/py_requirements.txt .
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install -y \
      --from-paths src \
      --ignore-src && \
    pip3 install -r py_requirements.txt && \
    rm -rf /var/lib/apt/lists/*

# ----------------------------

# Stage 3: Julia installation and environment setup
WORKDIR /root/julia_install/
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.11/julia-1.11.1-linux-x86_64.tar.gz \
  && tar zxvf julia-1.11.1-linux-x86_64.tar.gz -C /usr/local --strip-components=1 \
  && rm julia-1.11.1-linux-x86_64.tar.gz
ENV PATH="/usr/local/bin:$PATH"

# Set Julia and PythonCall/JuliaCall environment variables for consistency
ENV JULIA_VERSION=1.11.1
ENV JULIA_DEPOT_PATH=/root/.julia
ENV JULIA_PROJECT=/root/.julia/environments/pyjuliapkg
ENV PYTHON_JULIAPKG_EXE=/usr/local/bin/julia
ENV PYTHON_JULIAPKG_PROJECT=/root/.julia/environments/pyjuliapkg
ENV PYTHON_JULIAPKG_OFFLINE=yes
ENV PYTHON_JULIACALL_INSTALL=0

# Preinstall Julia packages into the correct project and depot, and precompile
COPY deps/install.jl /root/install.jl
RUN julia --project=${JULIA_PROJECT} /root/install.jl && \
  julia --project=${JULIA_PROJECT} -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()'

# ----------------------------
# Stage 4: ROS workspace build
# ----------------------------
WORKDIR $OVERLAY_WS
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin $OVERLAY_MIXINS

# ---- source entrypoint setup ----
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# Default working directory
WORKDIR $OVERLAY_WS
