# Run this ONE TIME
# docker buildx create --name builder_mp --use --bootstrap

# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib-opencv:1.0.0 -t bobcamera/boblib-opencv:latest --target opencv
# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib:1.0.0 -t bobcamera/boblib:latest --target boblib
# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.1 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.4 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev:1.0.6 -t bobcamera/bob-ros2-iron-dev:latest --target bob-ros2-iron-dev

# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/boblib-app:latest bash
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/boblib-opencv:latest bash

# --progress=plain this build switch will show entire build output
# --no-cache this build switch will rebuild and not use any cached output

# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:1.2.7 -t bobcamera/bob-opencv:latest --target opencv
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:1.2.7 -t bobcamera/bob-opencv:latest --target opencv
#############################################################################
# MWG: This docker stage is used to setup and build the OpenCV builder image
#############################################################################
FROM ubuntu:noble as opencv
ENV OPENCV_VERSION=4.10.0
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        wget \
        git \
        unzip \
        usbutils \
        libusb-1.0-0-dev \
        python3-dev \
        python3-numpy \
        python3-pip \
        qtbase5-dev \
        libdc1394-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libgstreamer-opencv1.0-0 \
        libgstreamer-plugins-base1.0-dev \
        libtbb-dev \
        libva-dev libva-drm2 libva-x11-2 libva-glx2 \
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && cd /tmp \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && cd /tmp \
    && wget --no-check-certificate -O opencv-$OPENCV_VERSION.zip https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip \
    && wget --no-check-certificate -O opencv_contrib-$OPENCV_VERSION.zip https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip \
    && unzip opencv-$OPENCV_VERSION.zip \
    && unzip opencv_contrib-$OPENCV_VERSION.zip \
    && rm -f opencv-$OPENCV_VERSION.zip \
    && rm -f opencv_contrib-$OPENCV_VERSION.zip \
    && cd opencv-$OPENCV_VERSION && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_GENERATE_PKGCONFIG=YES \
        -D ENABLE_PRECOMPILED_HEADERS=OFF \
        -D WITH_LIBV4L=ON \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D WITH_OPENCL=ON \
        -D WITH_JAVA=OFF \
        -D WITH_CUDA=OFF \
        -D BUILD_opencv_java=OFF \
        -D BUILD_opencv_python=OFF \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D PYTHON3_EXECUTABLE=$(which python3) \
        -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
        -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D BUILD_NEW_PYTHON_SUPPORT=ON \
        -D ENABLE_CXX11=ON \
        -D WITH_TBB=ON \
        -D BUILD_TESTS=OFF \
        -D INSTALL_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D INSTALL_C_EXAMPLES=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-$OPENCV_VERSION/modules \
        -D OpenGL_GL_PREFERENCE=LEGACY \
        .. \
    && make -j $(nproc) \
    && make install \
    && sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig \
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.12/


###################################################################
# MWG: This docker stage is used to build the QHY stuff
###################################################################
FROM ubuntu:noble as qhy
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
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-dev:1.2.7 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-dev:1.2.7 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
###################################################################
# MWG: This docker stage is used to build the ros2 dev environment
###################################################################
FROM ros:jazzy AS bob-ros2-dev
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.12/:/usr/local/lib/python3.12/dist-packages/
ENV PATH=$PATH:/root/.local/bin
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy the compiled libs
COPY --from=opencv /usr/local/lib/libopencv_* /usr/local/lib
COPY --from=opencv /usr/local/lib/cmake /usr/local/lib/cmake
COPY --from=opencv /usr/local/include /usr/local/include
COPY --from=opencv /usr/lib/python3 /usr/lib/python3
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
# install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
        libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 \
        libgstreamer-plugins-base1.0-0 libgstreamer-plugins-good1.0-0 libgstreamer-plugins-bad1.0-0 \
        libavcodec-extra60 libavformat-extra60 libswscale7 \
        locales tzdata sudo bash-completion libjsoncpp-dev libboost-python-dev libboost-system-dev libtbb-dev \
        pip pipx \
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
    && echo "export DISPLAY=:0" >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/${USERNAME}/.bashrc
# Building new ros-${ROS_DISTRO}-vision-opencv
WORKDIR /opt/ros2_ws
RUN mkdir -p /opt/ros2_ws/src \
   && git clone https://github.com/ros-perception/vision_opencv.git \
   && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
   && cd /opt/ros2_ws \
   && rosdep fix-permissions \
   && rosdep update \
   && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
   && colcon build"


# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-build:1.2.7 -t bobcamera/bob-ros2-build:latest --target bob-ros2-build
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
    && colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release"


# docker build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-prod:1.2.7 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-prod:1.2.7 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-prod:1.2.7 -t bobcamera/bob-ros2-prod:latest --target bob-ros2-prod
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v ~/Source/bobcamera/bobcamera/assets:/workspaces/bobcamera/src/ros2/assets/ bobcamera/bob-ros2-prod:1.2.3 bash

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
# Copy the compiled libs
COPY --from=opencv /usr/local/lib/libopencv_*.so /usr/local/lib
COPY --from=bob-ros2-build /usr/local/lib/libboblib.a /usr/local/lib
COPY --from=bob-ros2-build /usr/local/lib/python3.12/dist-packages/ /usr/local/lib/python3.12/dist-packages/
COPY --from=opencv /usr/lib/python3 /usr/lib/python3
COPY --from=opencv /usr/local/lib/libopencv_*.so /usr/local/lib
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
COPY media/fisheye_videos /workspaces/bobcamera/media/fisheye_videos
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/assets /workspaces/bobcamera/src/ros2/assets
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/install /workspaces/bobcamera/src/ros2/install
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/config /workspaces/bobcamera/src/ros2/config
COPY --from=bob-ros2-build /workspaces/bobcamera/src/ros2/launch* /workspaces/bobcamera/src/ros2/
COPY --from=bob-ros2-build /opt/ros2_ws/install/ /opt/ros2_ws/install/
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



# ###################################################################
# # https://lostindetails.com/articles/How-to-run-cron-inside-Docker#dockerfile
# FROM bobcamera/bob-ros2-dev AS bob-crontab
# COPY src/research /bob-research

# RUN apt-get update && apt-get install cron -y && \
#     chmod +x /bob-research/cron/run_heatmap.sh && \
#     chmod +x /bob-research/cron/test_crontab.sh

# ADD src/research/cron/crontab /etc/cron.d/bob-crontab

# RUN chmod 0644 /etc/cron.d/bob-crontab && \
#     crontab /etc/cron.d/bob-crontab

# # Creating entry point for cron
# ENTRYPOINT ["cron", "-f"]