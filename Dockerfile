ARG ARCH=arm32v7
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM duckietown/dt-ros-commons:${BASE_TAG}
RUN rm -r /custom_ws; mkdir /custom_ws
RUN /bin/bash -c "cat /etc/os-release"

WORKDIR /custom_ws

RUN apt-get update
RUN apt-get install --no-install-recommends -y software-properties-common
RUN apt-add-repository universe
RUN apt-get update
#RUN apt-get install python-pip

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    pkg-config \
    python \
    python-dev \
    python-setuptools \
    software-properties-common \
    unzip \
    git \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN curl https://bootstrap.pypa.io/2.7/get-pip.py --output get-pip.py
RUN python get-pip.py
RUN pip install wheel==0.34.1
#RUN path/to/pythonX.Y -m pip debug --verbose
RUN python -c "import wheel.pep425tags;print(wheel.pep425tags.get_supported("any"))"

#ARG WHL_URL=https://storage.googleapis.com/tensorflow/linux/cpu/
ARG WHL_URL=https://storage.googleapis.com/tensorflow/raspberrypi/

ARG WHL_FILE=tensorflow-1.14.0-cp27-none-linux_armv7l.whl
#ARG WHL_FILE=tensorflow-1.9.0-cp27-none-linux_x86_64.whl


RUN /bin/bash -c "add-apt-repository ppa:ubuntu-toolchain-r/test"
RUN /bin/bash -c "apt-get update"
RUN apt-get install -y libhdf5-dev
RUN apt-get install -y libzbar-dev libzbar0
RUN /bin/bash -c "apt-get -y install --no-install-recommends gcc-4.9 "
RUN /bin/bash -c "apt-get -y install --no-install-recommends libstdc++6"
RUN pip install h5py==2.9.0
RUN /bin/bash -c "pip2 install enum34 --ignore-installed enum34"

RUN python -m pip install --upgrade pip && \
  pip --no-cache-dir install \
     ipykernel \
     matplotlib \
     numpy \
     sklearn \
     pandas \
     ${WHL_URL}${WHL_FILE} && \
     rm -f ${WHL_FILE} && \
     python -m ipykernel.kernelspec 

#additional requirements
RUN pip install pyyaml
#RUN python -m pip install --upgrade pip && pip install opencv-python==4.3.0.38
#RUN pip install opencv-python
RUN apt-get install libopencv-dev
#RUN apt-get update && apt-get install -y python-opencv
#RUN pip install opencv-python



RUN pip install imageio==2.6.1
RUN pip install Cython
#RUN pip install cmake
#RUN pip install cmake
RUN pip install scikit-build
RUN pip install cmake==3.11.4

#RUN apt-get install cmake

RUN apt-get install gcc g++ 
RUN apt-get install python-dev
RUN apt-get install libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk2.0-dev
RUN apt-get install git


#RUN git clone https://github.com/opencv/opencv.git

#RUN cd opencv/ && mkdir build && cd build && cmake ../ && make && make install

#---------------------------------------------------
#OPENCV 3 (PYTHON2.7 SUPPORTED VERSION) INSTALLATION
RUN cmake --version
ENV OPENCV_VERSION 3.4.2
RUN wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip -O opencv3.zip &&  \
    unzip -q opencv3.zip && ls && \
    mv opencv-$OPENCV_VERSION /opencv && \
    rm opencv3.zip && \
    wget https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip -O opencv_contrib3.zip && \
    unzip -q opencv_contrib3.zip && \
    mv opencv_contrib-$OPENCV_VERSION /opencv_contrib && \
    rm opencv_contrib3.zip \
    && \

# Prepare build
    mkdir /opencv/build && cd /opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D BUILD_PYTHON_SUPPORT=ON \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF \
      -D WITH_IPP=OFF \
      -D WITH_FFMPEG=ON \
      -D WITH_V4L=ON .. \
    && \

# Install
    cd /opencv/build && \
    make -j$(nproc) && \
    make install && \
    ldconfig 
    
#----------------------------------------------------

#RUN pip install --upgrade pip && pip install opencv-python-headless==4.2.0.30
#RUN pip install imgaug==0.3.0
#RUN apt-get -y remove cmake 
#RUN pip install cmake

RUN pip install rospkg
RUN pip install defusedxml
RUN pip install Pillow


ENV HOSTNAME=agent
ENV VEHICLE_NAME=agent
ENV ROS_MASTER_URI=http://localhost:11311

RUN mkdir /custom_ws/src
RUN mkdir /src
RUN mkdir /custom_ws/src/object_detection
RUN mkdir /src/object_detection

COPY action_node custom_ws/src/action_node
COPY action_node/scripts/obstacles.py custom_ws/src/action_node/scripts/obstacles.py
COPY object_detection_dt/configuracion custom_ws/src/object_detection_dt/configuracion
COPY object_detection_dt/modelo_congelado custom_ws/src/object_detection_dt/modelo_congelado
COPY object_detection_dt/msg custom_ws/src/object_detection_dt/msg
COPY object_detection_dt/scripts custom_ws/src/object_detection_dt/scripts
COPY object_detection_dt/utils custom_ws/src/object_detection_dt/utils
COPY object_detection_dt/CMakeLists.txt custom_ws/src/object_detection_dt/CMakeLists.txt
COPY object_detection_dt/package.xml custom_ws/src/object_detection_dt/package.xml

RUN chmod +x custom_ws/src/object_detection_dt/scripts/object_detection_node.py
RUN chmod +x custom_ws/src/action_node/scripts/action_node1.py
RUN chmod +x custom_ws/src/action_node/scripts/path_planning_server.py

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    catkin build --workspace /custom_ws

ENV DISABLE_CONTRACTS=1




RUN catkin config -DPYTHON_EXECUTABLE=/usr/bin/python2.7 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython2.7.so
RUN catkin config --install

RUN echo "source custom_ws/devel/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make -j -C -DPYTHON_EXECUTABLE=/usr/bin/python2.7 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython2.7.so custom_ws/"





CMD ["/bin/bash", "-c", "export PYTHONPATH=$PYTHONPATH:$(pwd)/custom_ws/src/action_node/scripts:/usr/local/bin && source /custom_ws/custom_ws/devel/setup.bash && roslaunch action_node ads_program.launch"]