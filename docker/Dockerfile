# Run this ONE TIME
# docker buildx create --name builder_mp --use --bootstrap

# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib-opencv:1.0.0 -t bobcamera/boblib-opencv:latest --target opencv
# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib:1.0.0 -t bobcamera/boblib:latest --target boblib
# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.1 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.4 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev:1.0.6 -t bobcamera/bob-ros2-iron-dev:latest --target bob-ros2-iron-dev

# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/boblib-app:latest bash
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/boblib-opencv:latest bash

# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v .:/home/test pytorch/pytorch:2.4.1-cuda12.4-cudnn9-runtime bash

# --progress=plain this build switch will show entire build output
# --no-cache this build switch will rebuild and not use any cached output

# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:1.2.7 -t bobcamera/bob-opencv:latest --target opencv
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:1.5.7 -t bobcamera/bob-opencv:latest --target opencv

###################################################################
# MWG: This docker stage is used to build the QHY stuff
###################################################################
FROM ubuntu:noble AS qhy
RUN apt-get update && apt-get install -y --no-install-recommends wget tar \
    && cd /opt \
    && TMPDIR=/opt \
    && AMD64_LINK=https://www.qhyccd.com/file/repository/publish/SDK/230509/sdk_linux64_23.05.09.tgz \
    && ARM64_LINK=https://www.qhyccd.com/file/repository/publish/SDK/230509/sdk_Arm64_23.05.09.tgz \
    && if [ "$(uname -m)" = "x86_64" ]; then \
        DOWNLOAD_LINK=$AMD64_LINK; \
    else \
        DOWNLOAD_LINK=$ARM64_LINK; \
    fi \
    && wget --no-check-certificate  -q $DOWNLOAD_LINK -O $TMPDIR/sdk.tgz \
    && tar -xf $TMPDIR/sdk.tgz -C $TMPDIR \
    && mv $TMPDIR/sdk_* $TMPDIR/sdk_qhy \
    && cd $TMPDIR/sdk_qhy \
    && bash install.sh


# docker build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-dev:1.2.7 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --progress=plain --push --platform linux/amd64 -f ./docker/Dockerfile . -t bobcamera/bob-ros2-dev:1.5.8 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f ./docker/Dockerfile . -t bobcamera/bob-ros2-dev:1.2.7 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
###################################################################
# MWG: This docker stage is used to build the ros2 dev environment
###################################################################
FROM bobcamera/bob-opencv:1.5.8 AS bob-ros2-dev
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.12/:/usr/local/lib/python3.12/dist-packages/
ENV PATH=$PATH:/root/.local/bin
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy the compiled libs
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
COPY version.txt src/ros2/
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
        # libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 \
        # libavcodec-extra60 libavformat-extra60 libswscale7 \
        # locales tzdata sudo bash-completion libtbb-dev \
        # pip pipx \
        libjsoncpp-dev \
        libgstreamer-plugins-base1.0-0 libgstreamer-plugins-good1.0-0 libgstreamer-plugins-bad1.0-0 \
        libboost-python-dev libboost-system-dev locales \
        ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-image-transport \
        python3-pillow python3-pymongo python3-tornado python3-cairosvg \
    # Install ONVIF library so we can get camera details using the ONVIF protocol
    && mv /usr/lib/python3.12/EXTERNALLY-MANAGED /usr/lib/python3.12/EXTERNALLY-MANAGED.old \
    && pip install --upgrade onvif2_zeep \
    # Install QHY SDK
    && cd /opt/sdk_qhy && bash install.sh \
    # cleaning
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    # user related install
    && locale-gen en_GB.UTF-8 \
    && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    # Create ros user and modify exists uid ${USER_GID} if exists
    && if ! getent group ${USER_GID} > /dev/null; then \
        groupadd -g ${USER_GID} ${USERNAME}; \
    fi \
    && USER_WITH_UID=$(getent passwd ${USER_UID} | cut -d: -f1) \
    && if [ -n "$USER_WITH_UID" ]; then \
        usermod -u 10000 $USER_WITH_UID; \
    fi \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo 'export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' >> /home/${USERNAME}/.bashrc \
    && echo "export DISPLAY=:0" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/${USERNAME}/.bashrc
# Building new ros-${ROS_DISTRO}-vision-opencv
WORKDIR /opt/ros2_ws
RUN mkdir -p /opt/ros2_ws/src && cd /opt/ros2_ws/src \
   && git clone https://github.com/ros-perception/vision_opencv.git \
   && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
   && cd /opt/ros2_ws \
   && colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev"

# docker build --progress=plain --platform linux/amd64 -f ./docker/Dockerfile . -t bobcamera/bob-ros2-build:1.5.8 -t bobcamera/bob-ros2-build:latest --target bob-ros2-build
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/bob-ros2-build:1.2.7 bash
###############################################################
# MWG: This docker stage is used to build the ros2 application
###############################################################
FROM bob-ros2-dev AS bob-ros2-build
COPY src/ros2 /workspaces/bobcamera/src/ros2
COPY src/boblib /workspaces/bobcamera/src/boblib
WORKDIR /workspaces/bobcamera/src/ros2
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /opt/ros2_ws/install/setup.bash \
    && mkdir -p cd ../boblib/build \
    && cd ../boblib/build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && cmake --build . -j$(nproc) \
    && sudo make install \
    && cd - \
    && rm -rf log install build ..\boblib\build \
    && colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev"


# docker build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-prod:1.2.7 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-prod:1.2.7 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-prod:1.2.7 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker buildx build --progress=plain --platform linux/amd64 -f ./docker/Dockerfile . -t bobcamera/bob-ros2-prod:1.5.8 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v .:/workspaces/bobcamera/src/ros2/assets/ bobcamera/bob-ros2-prod:1.6.3 bash

###################################################################################
# MWG: This docker stage is used to construct the image with the build artifacts 
# outputted from the build stage above
###################################################################################
FROM ros:jazzy-ros-core AS bob-ros2-prod
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.12/:/usr/local/lib/python3.12/dist-packages/
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/version.txt /workspaces/bobcamera/src/ros2/
# Copy the compiled libs
COPY --from=bob-ros2-build /usr/local/lib /usr/local/lib
COPY --from=bob-ros2-build /usr/lib/python3 /usr/lib/python3
COPY --from=bob-ros2-build /usr/lib/x86_64-linux-gnu/libOpenCL* /usr/lib/x86_64-linux-gnu
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
#COPY media/fisheye_videos /workspaces/bobcamera/media/fisheye_videos
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/install /workspaces/bobcamera/src/ros2/install
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/config /workspaces/bobcamera/src/ros2/config
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/launch* /workspaces/bobcamera/src/ros2/
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/dual_camera_config.yaml /workspaces/bobcamera/src/ros2/
COPY --from=bob-ros2-build /opt/ros2_ws/install/ /opt/ros2_ws/install/

# Copying and configuring the CUDA stub to make Bob work
COPY --from=bob-ros2-build /usr/local/cuda/targets/x86_64-linux/lib/stubs/* /usr/local/cuda/targets/x86_64-linux/lib/
COPY --from=bob-ros2-build /usr/lib/wsl/lib/libcuda.so /usr/local/cuda/targets/x86_64-linux/lib/
#COPY --from=bob-ros2-build /usr/lib/wsl/lib/libdxcore.so /usr/local/cuda/targets/x86_64-linux/lib/
#COPY --from=bob-ros2-build /usr/lib/wsl/lib/libnvcuvid.so /usr/local/cuda/targets/x86_64-linux/lib/
#COPY --from=bob-ros2-build /usr/lib/wsl/lib/libnvidia-encode.so /usr/local/cuda/targets/x86_64-linux/lib/
RUN ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppc.so /usr/local/cuda/targets/x86_64-linux/lib/libnppc.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppicc.so /usr/local/cuda/targets/x86_64-linux/lib/libnppicc.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcuda.so /usr/local/cuda/targets/x86_64-linux/lib/libcuda.so.1 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnvcuvid.so /usr/local/cuda/targets/x86_64-linux/lib/libnvcuvid.so.1 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnvidia-encode.so /usr/local/cuda/targets/x86_64-linux/lib/libnvidia-encode.so.1 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppig.so /usr/local/cuda/targets/x86_64-linux/lib/libnppig.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppial.so /usr/local/cuda/targets/x86_64-linux/lib/libnppial.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppidei.so /usr/local/cuda/targets/x86_64-linux/lib/libnppidei.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppist.so /usr/local/cuda/targets/x86_64-linux/lib/libnppist.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppitc.so /usr/local/cuda/targets/x86_64-linux/lib/libnppitc.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppif.so /usr/local/cuda/targets/x86_64-linux/lib/libnppif.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcublas.so /usr/local/cuda/targets/x86_64-linux/lib/libcublas.so.12 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcufft.so /usr/local/cuda/targets/x86_64-linux/lib/libcufft.so.11 \
    && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppim.so /usr/local/cuda/targets/x86_64-linux/lib/libnppim.so.12 \
    && sh -c 'echo "/usr/local/cuda/targets/x86_64-linux/lib" >> /etc/ld.so.conf.d/cuda_stub.conf' && ldconfig

# install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 libjsoncpp-dev pip \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-vaapi \
    locales tzdata sudo bash-completion \
    libhdf5-dev libatlas-base-dev \
    libboost-python-dev libboost-system-dev libtbb-dev \        
    ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-image-transport \
    python3-pillow python3-pymongo python3-tornado python3-cairosvg \
    # Install ONVIF library so we can get camera details using the ONVIF protocol
    && mv /usr/lib/python3.12/EXTERNALLY-MANAGED /usr/lib/python3.12/EXTERNALLY-MANAGED.old \
    && pip install --upgrade onvif2_zeep \
    # Install QHY SDK
    && cd /opt/sdk_qhy && bash install.sh \
    # Install the libs locally
    && sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig \
    # cleaning
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /opt/sdk_qhy \
    # user related install
    && locale-gen en_GB.UTF-8 \
    && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    # Create ros user and modify exists uid ${USER_GID} if exists
    && if ! getent group ${USER_GID} > /dev/null; then \
        groupadd -g ${USER_GID} ${USERNAME}; \
    fi \
    && USER_WITH_UID=$(getent passwd ${USER_UID} | cut -d: -f1) \
    && if [ -n "$USER_WITH_UID" ]; then \
        usermod -u 10000 $USER_WITH_UID; \
    fi \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo 'export PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' >> /home/${USERNAME}/.bashrc \
    #&& echo 'export LD_LIBRARY_PATH=/usr/local/cuda/targets/x86_64-linux/lib:\$LD_LIBRARY_PATH' >> /home/${USERNAME}/.bashrc \
    && echo "export DISPLAY=:0" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/${USERNAME}/.bashrc
WORKDIR /workspaces/bobcamera/src/ros2
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bob_launch", "dynamic_launch.py"]




FROM nvidia/cuda:12.5.1-runtime-ubuntu24.04 AS bob-ros2-prod-cuda
################# ROS JAZZY CORE
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO jazzy
# setup timezone and install packages
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get -y upgrade && \
    apt-get install -q -y --no-install-recommends tzdata dirmngr gnupg2 && \
    rm -rf /var/lib/apt/lists/*
# setup keys and setup sources.list
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME" \
    && echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2-latest.list
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-core=0.11.0-1* \
    && rm -rf /var/lib/apt/lists/*
################# END ROS JAZZY CORE
################# BOB
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.12/:/usr/local/lib/python3.12/dist-packages/
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/version.txt /workspaces/bobcamera/src/ros2/
# Copy the compiled libs
COPY --from=bob-ros2-build /usr/local/lib /usr/local/lib
COPY --from=bob-ros2-build /usr/lib/python3 /usr/lib/python3
COPY --from=bob-ros2-build /usr/lib/x86_64-linux-gnu/libOpenCL* /usr/lib/x86_64-linux-gnu
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
#COPY media/fisheye_videos /workspaces/bobcamera/media/fisheye_videos
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/install /workspaces/bobcamera/src/ros2/install
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/config /workspaces/bobcamera/src/ros2/config
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/launch* /workspaces/bobcamera/src/ros2/
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/dual_camera_config.yaml /workspaces/bobcamera/src/ros2/
COPY --from=bob-ros2-build /opt/ros2_ws/install/ /opt/ros2_ws/install/

# Copying and configuring the CUDA stub to make Bob work
COPY --from=bob-ros2-build /usr/lib/wsl/lib/libcuda.so /usr/local/cuda/targets/x86_64-linux/lib/
COPY --from=bob-ros2-build /usr/lib/wsl/lib/libdxcore.so /usr/local/cuda/targets/x86_64-linux/lib/
COPY --from=bob-ros2-build /usr/lib/wsl/lib/libnvcuvid.so /usr/local/cuda/targets/x86_64-linux/lib/
COPY --from=bob-ros2-build /usr/lib/wsl/lib/libnvidia-encode.so /usr/local/cuda/targets/x86_64-linux/lib/
# RUN ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppc.so /usr/local/cuda/targets/x86_64-linux/lib/libnppc.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppicc.so /usr/local/cuda/targets/x86_64-linux/lib/libnppicc.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcuda.so /usr/local/cuda/targets/x86_64-linux/lib/libcuda.so.1 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnvcuvid.so /usr/local/cuda/targets/x86_64-linux/lib/libnvcuvid.so.1 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnvidia-encode.so /usr/local/cuda/targets/x86_64-linux/lib/libnvidia-encode.so.1 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppig.so /usr/local/cuda/targets/x86_64-linux/lib/libnppig.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppial.so /usr/local/cuda/targets/x86_64-linux/lib/libnppial.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppidei.so /usr/local/cuda/targets/x86_64-linux/lib/libnppidei.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppist.so /usr/local/cuda/targets/x86_64-linux/lib/libnppist.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppitc.so /usr/local/cuda/targets/x86_64-linux/lib/libnppitc.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppif.so /usr/local/cuda/targets/x86_64-linux/lib/libnppif.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcublas.so /usr/local/cuda/targets/x86_64-linux/lib/libcublas.so.12 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcufft.so /usr/local/cuda/targets/x86_64-linux/lib/libcufft.so.11 \
#     && ln -s /usr/local/cuda/targets/x86_64-linux/lib/libnppim.so /usr/local/cuda/targets/x86_64-linux/lib/libnppim.so.12 \
#     && sh -c 'echo "/usr/local/cuda/targets/x86_64-linux/lib" >> /etc/ld.so.conf.d/cuda_stub.conf' && ldconfig

# install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 libjsoncpp-dev pip \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-vaapi \
    locales tzdata sudo bash-completion \
    libhdf5-dev libatlas-base-dev \
    libboost-python-dev libboost-system-dev libtbb-dev \        
    ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-image-transport \
    python3-pillow python3-pymongo python3-tornado python3-cairosvg \
    # Install ONVIF library so we can get camera details using the ONVIF protocol
    && mv /usr/lib/python3.12/EXTERNALLY-MANAGED /usr/lib/python3.12/EXTERNALLY-MANAGED.old \
    && pip install --upgrade onvif2_zeep \
    # Install QHY SDK
    && cd /opt/sdk_qhy && bash install.sh \
    # Install the libs locally
    && sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig \
    # cleaning
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /opt/sdk_qhy \
    # user related install
    && locale-gen en_GB.UTF-8 \
    && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    # Create ros user and modify exists uid ${USER_GID} if exists
    && if ! getent group ${USER_GID} > /dev/null; then \
        groupadd -g ${USER_GID} ${USERNAME}; \
    fi \
    && USER_WITH_UID=$(getent passwd ${USER_UID} | cut -d: -f1) \
    && if [ -n "$USER_WITH_UID" ]; then \
        usermod -u 10000 $USER_WITH_UID; \
    fi \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo 'export PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' >> /home/${USERNAME}/.bashrc \
    #&& echo 'export LD_LIBRARY_PATH=/usr/local/cuda/targets/x86_64-linux/lib:\$LD_LIBRARY_PATH' >> /home/${USERNAME}/.bashrc \
    && echo "export DISPLAY=:0" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/${USERNAME}/.bashrc
WORKDIR /workspaces/bobcamera/src/ros2
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bob_launch", "dynamic_launch.py"]


# Navigate to http://localhost:8080 or http://127.0.0.1:8080 to view web output
# docker run -it -p 8080:80 bobcamera/bob-web-prod:1.2.7 bash
# docker build --progress=plain --push --platform linux/amd64 -f ./docker/Dockerfile . -t bobcamera/bob-web-prod:1.2.16 -t bobcamera/bob-web-prod:latest --target bob-web-prod
FROM php:8.3-apache AS bob-web-prod
COPY src/web2/src /var/www/html/
COPY src/web2/clear-cache.conf /etc/apache2/conf-available/
RUN ln -s /etc/apache2/conf-available/clear-cache.conf /etc/apache2/conf-enabled/clear-cache.conf
