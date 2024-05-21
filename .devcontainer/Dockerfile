# this dockerfile works as development environment for the course Modelling and Control of Legged Robots at TUM

##### Stage 1: Base Image #####

ARG FROM_IMAGE=ubuntu:focal
FROM $FROM_IMAGE as base

LABEL MAINTAINER="yueyang.zhang@tum.de"

SHELL [ "/bin/bash", "-c" ]

# setup environment
ENV LANG=en_US.UTF-8
ENV LC_NUMERIC=en_US.UTF-8
ENV LC_ALL=C
ENV DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -q -y --no-install-recommends tzdata && \
  rm -rf /var/lib/apt/lists/*

# install develop packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  git \
  net-tools \
  openssl \
  curl \
  wget \
  zsh \
  mesa-utils \
  build-essential \
  vim \
  bash-completion \ 
  software-properties-common \
  ca-certificates \
  ssh \
  dirmngr \
  gnupg2 \
  lsb-release \
  python3.8\
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

##### Stage 2: ROS #####
# install ros
FROM base as ros

ENV ROS_DISTRO noetic

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install -y ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt-get update && apt-get install --no-install-recommends -y \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  python3-vcstools \
  python3-catkin-tools\
  && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
  && rosdep update

##### Stage 3: User #####

FROM ros as user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -q -y sudo \
  && rm -rf /var/lib/apt/lists/* \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/developer/Downloads/openssl/ssl \
  && sudo cp -r /etc/ssl/* /home/developer/Downloads/openssl/ssl/ \
  && rosdep update