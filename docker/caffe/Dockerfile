FROM ros:indigo-cuda

ENV PYTHONPATH /opt/caffe/python

# Add caffe binaries to path
ENV PATH $PATH:/opt/caffe/.build_release/tools

# Get dependencies
RUN apt-get update && apt-get install -q -y \
  libprotobuf-dev \
  libleveldb-dev \
  libsnappy-dev \
  libopencv-dev \
  libhdf5-serial-dev \
  protobuf-compiler \
  libboost-all-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  liblmdb-dev \
  libatlas-base-dev \
  python-dev \
  && rm -rf /var/lib/apt/lists/*

# Allow it to find CUDA libs
RUN echo "/usr/local/cuda/lib64" > /etc/ld.so.conf.d/cuda.conf && \
  ldconfig

# Clone the Caffe repo
RUN cd /opt && git clone https://github.com/BVLC/caffe.git

# Build Caffe core
RUN cd /opt/caffe && \
  cp Makefile.config.example Makefile.config && \
  make all -j$(nproc)

# Add ld-so.conf so it can find libcaffe.so
ADD caffe-ld-so.conf /etc/ld.so.conf.d/

# Run ldconfig again (not sure if needed)
RUN ldconfig
