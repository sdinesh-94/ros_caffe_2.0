FROM osrf/ros:indigo-desktop-full
# A docker container with the Nvidia kernel module and CUDA drivers installed

# Set CUDA versions to use
ENV CUDA_V_ 7_5
ENV CUDA_V 7.5
ENV CUDA_VERSION 7.5.18
ENV CUDA_INSTALL http://developer.download.nvidia.com/compute/cuda/7.5/Prod/local_installers/cuda_7.5.18_linux.run

# Install some tools
RUN apt-get update && apt-get install -q -y \
    wget \
    build-essential \
    module-init-tools \
  && rm -rf /var/lib/apt/lists/*

# Download and Install CUDA
RUN cd /tmp \
  && wget $CUDA_INSTALL \
  && chmod +x cuda*.run \
  && mkdir nvidia_installer \
  && ./cuda*.run -extract=/tmp/nvidia_installers \
  && cd nvidia_installers \
  && ./NVIDIA*.run \
    --silent \
    --no-network \
    --no-kernel-module \
  && ./cuda*.run \
    -noprompt \
  && rm -rf /tmp/*

# Ensure the CUDA libs and binaries are in the correct environment variables
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
ENV PATH=$PATH:/usr/local/cuda/bin
RUN ldconfig

ADD cudnn-7.0-linux-x64-v3.0-prod.tgz /cudnn/cudnn-7.0-linux-x64-v3.0-prod
RUN cd /cudnn/cudnn*/cuda \
  && cp -r lib*/* /usr/local/cuda/lib64/ \
  && cp include/cudnn.h /usr/local/cuda/include/
