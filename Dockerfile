ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

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
RUN apt-get update && apt-get install -y python3-pip
RUN apt-get update && apt-get install -y tzdata
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

# # Install Julia
# RUN apt-get update && apt-get install -y curl
# RUN curl -fsSL https://install.julialang.org | sh -s -- --yes --add-to-path=yes

# # initialize juliacall
# COPY deps/juliapkg.json .
# # COPY deps/install.jl .
# RUN python3 -c 'from juliacall import Main as jl'
# # RUN julia install.jl

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

# Convert shell into bash
SHELL [ "/bin/bash", "-c" ]

# Create a wrapper for Python with the -X flag
# RUN echo '#!/bin/bash/nexec python3 -X juliapkg-project=$(pwd) "$@"' > /usr/local/bin/python-wrapper && \
#     chmod +x /usr/local/bin/python-wrapper

# Use the wrapper script as the default Python command
# RUN ln -sf /usr/local/bin/python-wrapper /usr/bin/python3

# Install Julia
RUN apt-get update && apt-get install -y curl wget vim
# RUN curl -fsSL https://install.julialang.org | sh -s -- --yes --add-to-path=yes
WORKDIR /root/julia_install/
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.11/julia-1.11.1-linux-x86_64.tar.gz && tar zxvf julia-1.11.1-linux-x86_64.tar.gz
RUN echo PATH="\$PATH:/root/julia_install/julia-1.11.1/bin" >> ~/.bashrc

COPY deps/Project.toml .
COPY deps/test.py .
COPY deps/juliapkg.json .
# RUN /root/julia_install/julia-1.11.1/bin/julia --project -e 'using Pkg; Pkg.instantiate()'
# RUN /root/julia_install/bin  /julia --project -e 'using Pkg; Pkg.instantiate()'
# RUN /root/.juliaup/bin/julia --project -e 'using Pkg; Pkg.instantiate()'
# RUN python3 -X juliapkg-project=$(pwd) -c 'from juliacall import Main as jl'
RUN python3 -c 'from juliacall import Main as jl'


# run launch file
# CMD ["ros2", "launch", "asv_sensors", "sensor_drivers.xml"]