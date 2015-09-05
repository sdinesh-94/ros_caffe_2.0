FROM ros:indigo-caffe

# Get dependencies
RUN apt-get update && apt-get install -q -y \
  ros-indigo-openni2-launch \
  ros-indigo-openni2-camera \
  && rm -rf /var/lib/apt/lists/*
