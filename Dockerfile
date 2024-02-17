# Run this ONE TIME
# docker buildx create --name builder_mp --use --bootstrap

# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib-opencv:1.0.0 -t bobcamera/boblib-opencv:latest --target opencv
# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib:1.0.0 -t bobcamera/boblib:latest --target boblib
# docker buildx build --push --platform linux/amd64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.1 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.4 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev:1.0.6 -t bobcamera/bob-ros2-iron-dev:latest --target bob-ros2-iron-dev

# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/boblib-app:latest bash
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/boblib-opencv:latest bash


# MWG 2024.02.10 Verisoned all containers to 2.1.0 as I need to test this stuff and don't want to interfere with existing containers
# --progress=plain this build switch will show entire build output
# --no-cache this build switch will rebuild and not use any cached output

# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:2.1.0 -t bobcamera/bob-opencv:latest --target opencv
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:2.1.0 -t bobcamera/bob-opencv:latest --target opencv
###################################################################
# MWG: This docker stage is used to build OpenCV
###################################################################
FROM ubuntu:22.04 as builder
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
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev \
        libatlas-base-dev \
        python3-dev \
        python3-numpy \
        python3-pip \
        libtbb2 \
        libtbb-dev \
        libdc1394-25 \
        libdc1394-dev \
        libopenexr-dev \
        libunwind-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-opencv1.0-0 \
        libgstreamer1.0-dev \
        liblapacke-dev \
        libva-dev libva-drm2 libva-x11-2 libva-glx2 \
        libhdf5-dev \
        qtbase5-dev \
        usbutils \
        libusb-1.0-0-dev \
    && ln -s /usr/bin/python3 /usr/bin/python \
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

###################################################################
FROM builder AS opencv
ENV OPENCV_VERSION=4.9.0
RUN cd /tmp \
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
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.10/




###################################################################
# MWG: This docker stage is used to build the QHY stuff
###################################################################
FROM ubuntu:22.04 as qhy
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






# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib:2.1.0 -t bobcamera/boblib:latest --target boblib
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib:2.1.0 -t bobcamera/boblib:latest --target boblib
###################################################################
# MWG: This docker stage is used to build BobLib Library
###################################################################
FROM opencv as boblib
COPY . /opt/bobcamera
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
RUN cd /opt/sdk_qhy && bash install.sh \
    && cd /opt \
    #&& GIT_SSL_NO_VERIFY=true git clone --recursive https://github.com/bobcamera/bobcamera.git \
    && mkdir -p /opt/bobcamera/src/boblib/build \
    && cd /opt/bobcamera/src/boblib/build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && cmake --build . -j$(nproc) \
    && make install
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages/




# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-boblibapp:2.1.0 -t bobcamera/bob-boblibapp:latest --target boblib-app
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-boblibapp:2.1.0 -t bobcamera/bob-boblibapp:latest --target boblib-app
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-boblibapp:2.1.0 -t bobcamera/bob-boblibapp:latest --target boblib-app
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/bob-boblibapp:latest bash
###################################################################
# MWG: This docker stage is used to build BobLib Application
###################################################################
FROM ubuntu:22.04 AS boblib-app
# Copy the compiled libs
COPY --from=boblib /usr/local/ /usr/local/
COPY --from=boblib /opt/bobcamera/src/boblib/install_app.sh /root
COPY --from=boblib /usr/lib/python3 /usr/lib/python3
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-vaapi \
        # libavcodec58 libavformat58 libswscale5 \
        # liblapack3 libatlas-base-dev openexr libhdf5-dev \
    # Install QHY SDK
    && cd /opt/sdk_qhy && bash install.sh \
    # Install the libs locally
    && sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig \
    # cleaning
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.10/:/usr/local/lib/python3/dist-packages/
WORKDIR /root



# docker build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev2:2.1.2 -t bobcamera/bob-ros2-iron-dev2:latest --target bob-ros2-iron-dev
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev2:2.1.0 -t bobcamera/bob-ros2-iron-dev2:latest --target bob-ros2-iron-dev
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev2:2.1.0 -t bobcamera/bob-ros2-iron-dev2:latest --target bob-ros2-iron-dev
###################################################################
# MWG I think the docker build script below has been used to build:
#                  *****bob-ros2-iron-dev2*****
# This is due to the docker buildx command that can be seen below
###################################################################
FROM ros:iron AS bob-ros2-iron-dev
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.10/:/usr/local/lib/python3/dist-packages/:/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV DEBIAN_FRONTEND=noninteractive
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy the compiled libs
COPY --from=boblib /usr/local/lib/libopencv_* /usr/local/lib
COPY --from=boblib /usr/local/lib/libboblib.a /usr/local/lib
COPY --from=boblib /usr/local/lib/cmake /usr/local/lib/cmake
COPY --from=boblib /usr/local/include /usr/local/include
COPY --from=boblib /usr/local/lib/python3/dist-packages/ /usr/local/lib/python3/dist-packages/
COPY --from=boblib /usr/lib/python3 /usr/lib/python3
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
# install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
        libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 libgstreamer-plugins-base1.0-0 \
        libavcodec58 libavformat58 libswscale5 liblapack3 libatlas-base-dev openexr libhdf5-dev \
        locales tzdata sudo bash-completion libjsoncpp-dev libboost-python-dev libboost-system-dev libtbb-dev pip \
        ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-image-transport \
    && pip install Pillow \
    && pip install pymongo \
    && pip install tornado \
    # Install ONVIF library so we can get camera details using the ONVIF protocol
    && pip install onvif2_zeep \
    # Install the library used to convert the svg mask to a PNG and then a JPG
    && pip install cairosvg \    
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
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "export DISPLAY=:0" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
# Building new ros-${ROS_DISTRO}-vision-opencv
WORKDIR /opt/ros2_ws
RUN mkdir -p /opt/ros2_ws/src \
   && git clone https://github.com/ros-perception/vision_opencv.git \
   && bash /opt/ros/${ROS_DISTRO}/setup.bash \
   && rosdep update \
   && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
   && colcon build


# docker build --progress=plain --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-build2:2.1.0 -t bobcamera/bob-ros2-iron-build2:latest --target bob-ros2-iron-build
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/bob-ros2-iron-build2:2.1.0 bash
###############################################################
# MWG: This docker stage is used to build the ros2 application
###############################################################
FROM bob-ros2-iron-dev AS bob-ros2-iron-build
COPY src/ros2 /workspaces/bobcamera/src/ros2
WORKDIR /workspaces/bobcamera/src/ros2
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # Nano for debug purposes
    # nano \
    && bash /opt/ros/$ROS_DISTRO/setup.bash \
    && bash /opt/ros2_ws/install/setup.bash \
    && colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release





# docker build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-prod2:2.1.4 -t bobcamera/bob-ros2-iron-prod2:latest --target bob-ros2-iron-prod
# docker buildx build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-prod2:2.1.0 -t bobcamera/bob-ros2-iron-prod2:latest --target bob-ros2-iron-prod
# docker buildx build --progress=plain --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-iron-prod2:2.1.0 -t bobcamera/bob-ros2-iron-prod2:latest --target bob-ros2-iron-prod
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v ~/Source/bobcamera/bobcamera/assets:/workspaces/bobcamera/src/ros2/assets/ bobcamera/bob-ros2-iron-prod2:2.1.0 bash
###################################################################################
# MWG: This docker stage is used to construct the image with the build artifacts 
# outputted from the build stage above
###################################################################################
FROM ros:iron-ros-core AS bob-ros2-iron-prod
#FROM ros:iron AS bob-ros2-iron-prod
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.10/:/usr/local/lib/python3/dist-packages/:/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV DEBIAN_FRONTEND=noninteractive
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy the compiled libs
COPY --from=boblib /usr/local/lib/libopencv_*.so /usr/local/lib
COPY --from=boblib /usr/local/lib/libboblib.a /usr/local/lib
# COPY --from=boblib /usr/local/lib/cmake /usr/local/lib/cmake
# COPY --from=boblib /usr/local/include /usr/local/include
COPY --from=boblib /usr/local/lib/python3/dist-packages/ /usr/local/lib/python3/dist-packages/
COPY --from=boblib /usr/lib/python3 /usr/lib/python3
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
COPY media/fisheye_videos /workspaces/bobcamera/media/fisheye_videos
COPY --from=bob-ros2-iron-build /workspaces/bobcamera/src/ros2/assets /workspaces/bobcamera/src/ros2/assets
COPY --from=bob-ros2-iron-build /workspaces/bobcamera/src/ros2/install /workspaces/bobcamera/src/ros2/install
COPY --from=bob-ros2-iron-build /workspaces/bobcamera/src/ros2/config /workspaces/bobcamera/src/ros2/config
COPY --from=bob-ros2-iron-build /workspaces/bobcamera/src/ros2/launch* /workspaces/bobcamera/src/ros2/
COPY --from=bob-ros2-iron-build /opt/ros2_ws/install/ /opt/ros2_ws/install/
# install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # Nano for debug purposes
    #nano \
    libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 libjsoncpp-dev pip \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-vaapi \
    locales tzdata sudo bash-completion \
    libhdf5-dev libatlas-base-dev \
    libboost-python-dev libboost-system-dev libtbb-dev \        
    ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-image-transport \
    && pip install Pillow \
    && pip install pymongo \
    && pip install tornado \
    # Install ONVIF library so we can get camera details using the ONVIF protocol
    && pip install onvif2_zeep \
    # Install the library used to convert the svg mask to a PNG and then a JPG
    && pip install cairosvg \    
    # Install QHY SDK
    && cd /opt/sdk_qhy && bash install.sh \
    # Install the libs locally
    && sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig \
    # cleaning
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /opt/sdk_qhy \
    # user related install
    # && locale-gen en_GB.UTF-8 \
    # && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 \
    # && dpkg-reconfigure --frontend noninteractive tzdata \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "export DISPLAY=:0" >> /home/$USERNAME/.bashrc
    # && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
WORKDIR /workspaces/bobcamera/src/ros2
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bob_launch", "application_launch.py"]


# Navigate to http://localhost:8080 or http://127.0.0.1:8080 to view web output
# docker run -it -p 8080:80 bobcamera/bob-web-prod:1.2.0 bash
# docker build --progress=plain --push --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-web-prod:1.2.1 -t bobcamera/bob-web-prod:latest --target bob-web-prod
FROM php:8.3-apache AS bob-web-prod
RUN apt-get update && apt-get install -y ffmpeg && apt-get clean
COPY src/web2/src /var/www/html/



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

