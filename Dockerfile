# docker build -t ros2:dashing .
FROM ubuntu:latest
ENV DEBIAN_FRONTEND noninteractive

RUN apt update -y && \
    apt install -y locales && \
    locale-gen ja_JP ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8

ENV LC_ALL   ja_JP.UTF-8
ENV LANG     ja_JP.UTF-8
ENV LANGUAGE ja_JP.UTF-8

RUN apt update -y && \
    apt install -y curl gnupg2 lsb-release && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update -y && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update -y && \
    apt install -y ros-humble-ros-base && \
    apt install -y ros-dev-tools && \
    apt install -y debhelper && \
    apt install -y devscripts && \
    apt install -y vim && \
    apt install -y python3-dev && \
    apt install -y dh-python && \

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
CMD ["bash"]
