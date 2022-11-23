ARG ROS_DISTRO=noetic

FROM ros:${ROS_DISTRO}-ros-core AS build-env
ENV DEBIAN_FRONTEND=noninteractive \
    BUILD_HOME=/var/lib/build \
    OUSTER_ROS_PATH=/opt/ouster_lidar_driver

RUN set -xue \
# Kinetic and melodic have python3 packages but they seem to conflict
&& [ $ROS_DISTRO = "noetic" ] && PY=python3 || PY=python \
# Turn off installing extra packages globally to slim down rosdep install
&& echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/01norecommend \
&& apt-get update \
&& apt-get install -y \
 build-essential cmake git \
 fakeroot dpkg-dev debhelper \
 $PY-rosdep $PY-rospkg $PY-bloom

# Set up non-root build user
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -o -g ${BUILD_GID} build \
&& useradd -o -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

# Install build dependencies using rosdep
COPY --chown=build:build ouster_ros/package.xml ${OUSTER_ROS_PATH}/ouster_ros/package.xml
COPY --chown=build:build cav_msgs/package.xml ${OUSTER_ROS_PATH}/cav_msgs/package.xml

RUN set -xe \
&& apt-get update \
&& rosdep init \
&& rosdep update --rosdistro=${ROS_DISTRO} \
&& rosdep install -y --from-paths ${OUSTER_ROS_PATH}

RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it &&\
    sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin

# Set up build environment
COPY --chown=build:build ouster_ros ${OUSTER_ROS_PATH}/ouster_ros
COPY --chown=build:build cav_msgs ${OUSTER_ROS_PATH}/cav_msgs

USER build:build
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& mkdir src \
&& ln -s ${OUSTER_ROS_PATH} ./src

FROM build-env

ENV CXXFLAGS="-Werror -Wno-deprecated-declarations"
RUN /opt/ros/${ROS_DISTRO}/env.sh catkin_make -DCMAKE_BUILD_TYPE=Release \
&& /opt/ros/${ROS_DISTRO}/env.sh catkin_make install

# Entrypoint for running Ouster ROS:
#
# Usage: docker run --rm -it ouster-ros [ouster.launch parameters ..]
#
CMD ["bash", "-c", "set -e \
&& . ./devel/setup.bash \
&& roslaunch ouster_ros ouster.launch \"$@\" \
", "ros-entrypoint"]