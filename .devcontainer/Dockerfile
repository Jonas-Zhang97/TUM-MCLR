# this dockerfile works as development environment for the course Modelling and Control of Legged Robots at TUM

##### Stage 1: Base Image #####


ARG BaseImage=nvidia/cuda:12.2.2-cudnn8-devel-ubuntu20.04
FROM $BaseImage as base

LABEL MAINTAINER="yueyang.zhang@tum.de"

SHELL [ "/bin/bash", "-c" ]

# setup environment
ENV LANG=en_US.UTF-8
ENV LC_NUMERIC=en_US.UTF-8
ENV LC_ALL=C
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# USE German mirror to speed up things
RUN cp /etc/apt/sources.list /etc/apt/sources.list.old \
  && sed -i -e 's/http:\/\/archive\.ubuntu\.com\/ubuntu\// \
  http:\/\/de.archive\.ubuntu\.com\/ubuntu/' /etc/apt/sources.list
RUN apt-key update && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -q -y --no-install-recommends tzdata && \
  rm -rf /var/lib/apt/lists/*

# install develop packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  mlocate \
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

# Fix ssl certificate issue
RUN apt-get update
RUN apt-get install ca-cacert \
  && update-ca-certificates \
  && rm -rf /var/lib/apt/lists/*

##### Stage 3: ROS #####
# install ros
FROM base as ros

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO noetic

RUN sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
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

##### Stage 2: User #####

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

RUN sudo mkdir -p /home/$USERNAME/Downloads/openssl/ssl \
  && sudo cp -r /etc/ssl/* /home/$USERNAME/Downloads/openssl/ssl/ \
  && sudo rm /etc/ros/rosdep/sources.list.d/20-default.list \
  && sudo rosdep init \
  && rosdep update

##### Stage 4: Development Environment #####
FROM user as deps

ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN  sudo apt install -qqy lsb-release curl

RUN sudo mkdir -p /etc/apt/keyrings
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
  | sudo tee /etc/apt/keyrings/robotpkg.asc

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
  | sudo tee /etc/apt/sources.list.d/robotpkg.list

RUN sudo apt update

RUN  sudo apt install -qqy robotpkg-py3*-pinocchio

RUN echo "export PATH=/opt/openrobots/bin:\$PATH" >> ~/.bashrc 
RUN echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH"  >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH"  >> ~/.bashrc
RUN echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:\$PYTHONPATH"  >> ~/.bashrc
RUN echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> ~/.bashrc

RUN pip install pybullet pandas
RUN pip install numpy --upgrade

RUN sudo apt-get update
RUN sudo apt-get install -y -q --no-install-recommends robotpkg-py3*-tsid
RUN echo "source /workspaces/mclr/mclr_ws/devel/setup.bash" >> ~/.bashrc

# for drake
# RUN sudo apt install python3.8-venv
# RUN python3 -m venv t7_env
# RUN t7_env/bin/pip install pip --upgrade
# RUN t7_env/bin/pip/install drake