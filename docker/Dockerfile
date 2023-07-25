# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile-simple2 . -t sky360/sky360lib-app:1.0.0 -t sky360/sky360lib-app:latest --target sky360lib-app
# docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile-simple2 . -t sky360/ros2-dev:1.0.4 -t sky360/ros2-dev:latest --target ros2-dev

# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY sky360/sky360lib-app:latest bash

# Builder
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
    # && wget --no-check-certificate -O get-pip.py https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py \
        #nano \
        #curl \
        #libgtk-3-dev \
        #libjpeg-dev \
        #libpng-dev \
        #libtiff-dev \
        #gfortran \
        #openexr \
        #qtchooser \
        #qt5-qmake \
        #qtbase5-dev-tools \
        #libvtk9-dev libvtk9-qt-dev \
        #clang \

###################################################################
FROM builder AS opencv
ENV OPENCV_VERSION=4.8.0
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
# FROM opencv as bgs
# RUN cd /opt && git clone --recursive https://github.com/Sky360-Repository/bgslibrary.git && \
#     mkdir -p /opt/bgslibrary/build && cd /opt/bgslibrary/build && \
#     cmake .. && make -j $(nproc) && \
#     cd /opt/bgslibrary && \
#     python setup.py build && \
#     python setup.py install && \
#     mkdir -p /opt/bgslibrary/gui/qt/build && \
#     cd /opt/bgslibrary/gui/qt/build && \
#     cmake .. && make -j $(nproc) && \
#     cp bgslibrary_gui ../../..
# ENV PYTHONPATH=$PYTHONPATH:/opt/bgslibrary/build/lib.linux-x86_64-cpython-310:/opt/bgslibrary/build/lib.linux-aarch64-cpython-310

###################################################################
FROM ubuntu:22.04 as qhy
RUN apt-get update && apt-get install -y --no-install-recommends wget tar \
    && cd /opt \
    && TMPDIR=/opt \
    && AMD64_LINK=https://www.qhyccd.com/file/repository/publish/SDK/230228/sdk_linux64_23.02.28.tgz \
    && ARM64_LINK=https://www.qhyccd.com/file/repository/publish/SDK/230228/sdk_Arm64_23.02.28.tgz \
    # detect
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

###################################################################
FROM opencv as sky360lib
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
RUN cd /opt/sdk_qhy && bash install.sh \
    && cd /opt \
    && GIT_SSL_NO_VERIFY=true git clone --recursive https://github.com/Sky360-Repository/sky360lib.git \
    && mkdir -p /opt/sky360lib/build \
    && cd /opt/sky360lib/build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && cmake --build . -j$(nproc) \
    && make install
ENV PYTHONPATH=$PYTHONPATH:/opt/sky360lib/build/lib

###################################################################
FROM ubuntu:22.04 AS sky360lib-app
# Copy the compiled libs
COPY --from=sky360lib /usr/local/ /usr/local/
COPY --from=qhy /opt/sdk_qhy /opt/sdk_qhy/
# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        libtbb12 libqt5opengl5 libqt5test5 libdc1394-25 libgstreamer-plugins-base1.0-0 \
        libavcodec58 libavformat58 libswscale5 liblapack3 libatlas-base-dev openexr \
    # Install QHY SDK 
    && cd /opt/sdk_qhy && bash install.sh \
    # Install the libs locally
    && sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig \
    # cleaning
    && apt-get autoclean && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/cv2/python-3.10/:/opt/sky360lib/build/lib


###################################################################
FROM sky360lib-app AS ros2-dev
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=iron
ENV LANG=en_GB.UTF-8
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      pkg-config \
      libtbb-dev \
      python3-dev \
      python3-numpy \
      python3-pip \
      python3-argcomplete \
      wget \
      git \
      libusb-1.0-0-dev \
      locales \
      curl \
      gnupg2 \
      lsb-release \
      sudo \
      tzdata \
      bash-completion \
      libboost-python-dev \
      libboost-system-dev \
   && locale-gen en_GB.UTF-8 \
   && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 \
   && dpkg-reconfigure --frontend noninteractive tzdata \
   && ln -s /usr/bin/python3 /usr/bin/python \
   && apt-get autoclean && apt-get clean \
   && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp \
   && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
#    && curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg \
   && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
#    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null \
   && apt-get update && apt-get install -y \
         ros-${ROS_DISTRO}-ros-base \
         ros-${ROS_DISTRO}-rosbridge-server \
         ros-${ROS_DISTRO}-rqt* \
         ros-${ROS_DISTRO}-vision-msgs \
         python3-argcomplete \
         python3-vcstool \
         python3-rosdep \
         python3-colcon-common-extensions \
        #  vulcanexus-humble-base \
   && rm -rf /var/lib/apt/lists/* \
#    # Build eProsima / prometheus-cpp --> https://github.com/eProsima/prometheus-cpp/
#    && cd /tmp \
#    && git clone https://github.com/eProsima/prometheus-cpp.git \
#    && cd /tmp/prometheus-cpp \
#    && git submodule init && git submodule update && mkdir build \
#    && cd /tmp/prometheus-cpp/build \
#    && cmake .. -DBUILD_SHARED_LIBS=ON -DENABLE_PUSH=OFF -DENABLE_COMPRESSION=OFF \
#    && cmake --build . -j $(nproc) \
#    && cmake --install . \
   # 
   && rosdep init || echo "rosdep already initialized" \
   && groupadd --gid $USER_GID $USERNAME \
   && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
   # [Optional] Add sudo support for the non-root user
   && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
   && chmod 0440 /etc/sudoers.d/$USERNAME \
   && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
   && echo "export DISPLAY=:0" >> /home/$USERNAME/.bashrc \
   && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc 
#    && echo "if [ -f /opt/vulcanexus/${ROS_DISTRO}/setup.bash ]; then source /opt/vulcanexus/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=$PYTHONPATH:/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
# Building new ros-${ROS_DISTRO}-vision-opencv
WORKDIR /opt/ros2_ws
RUN mkdir -p /opt/ros2_ws/src \
   && git clone https://github.com/ros-perception/vision_opencv.git \
   && rosdep update \
   && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --allow-overriding cv_bridge \
   && colcon build
ENV DEBIAN_FRONTEND=