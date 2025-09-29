ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

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

FROM $FROM_IMAGE AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=US/Eastern

# ---- System dependencies ----
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
RUN apt-get update && apt-get install -y \
      libssl-dev ca-certificates \
      python3-pip curl wget vim tzdata ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install juliacall numpy tzlocal\
    && python3 -c 'import juliacall'

COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && apt-get install -y\ 
      ros-$ROS_DISTRO-foxglove-bridge \
    && rosdep install -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# ----------------------
# Install Julia binary
# ----------------------
WORKDIR /root/julia_install
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.11/julia-1.11.1-linux-x86_64.tar.gz \
    && tar zxvf julia-1.11.1-linux-x86_64.tar.gz \
    && ln -s /root/julia_install/julia-1.11.1/bin/julia /usr/local/bin/julia

# ----------------------
# Julia environment (mounted .julia)
# ----------------------
ENV JULIA_DEPOT_PATH=/root/.julia
ENV JULIA_PROJECT=/root/.julia/environments/pyjuliapkg
ENV PYTHON_JULIAPKG_EXE=/usr/local/bin/julia
#ENV PYTHON_JULIAPKG_OFFLINE=yes
ENV PYTHON_JULIAPKG_OFFLINE=no
ENV PATH="/usr/local/bin:$PATH"

# Build julia
COPY deps/install.jl .
RUN julia -e 'using Pkg; \
		Pkg.activate("/root/.julia/environments/pyjuliapkg"); \
		include("install.jl")'

# Rebuild OpenSSL_jll to match container's libssl
#RUN julia -e 'using Pkg; Pkg.activate("/root/.julia/environments/pyjuliapkg")'

SHELL ["/bin/bash", "-c"]

# ----------------------
# Overlay workspace
# ----------------------
WORKDIR $OVERLAY_WS
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin $OVERLAY_MIXINS

# ----------------------
# Source ROS overlay entrypoint
# ----------------------
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# Default working directory
WORKDIR $OVERLAY_WS
