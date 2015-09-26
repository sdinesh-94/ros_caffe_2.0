FROM ros:indigo-ros-caffe

# Get dependencies
RUN apt-get update && apt-get install -q -y \
	curl \
	wget \
	screen \
	byobu \
	fish \
	git \
	nano \
	glances \
  terminator \
  bash-completion

RUN echo 'if [ -f /etc/bash_completion ] && ! shopt -oq posix; then\n\
    . /etc/bash_completion\n\
fi\n'\
>> ~/.bashrc
