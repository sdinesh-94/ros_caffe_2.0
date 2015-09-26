FROM ros:indigo-caffe

# Get dependencies
RUN apt-get update && apt-get install -q -y \
  ros-indigo-openni2-launch \
  ros-indigo-openni2-camera \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# clone omnimapper project
RUN mkdir -p catkin_ws/src \
  && cd catkin_ws/src \
  && git clone https://github.com/ruffsl/ros_caffe.git

# build omnimapper ros wrapper
ADD build.sh catkin_ws/src/ros_caffe/build.sh
RUN ./catkin_ws/src/ros_caffe/build.sh
