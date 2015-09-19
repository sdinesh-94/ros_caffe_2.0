#!/bin/sh

docker run \
	-it \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="/home/${USER}/Desktop/caffe:/root/Desktop" \
	--volume="/lib/modules/:/lib/modules/:ro" \
	--volume="/etc/modprobe.conf/:/etc/modprobe.conf/:ro" \
	--device /dev/nvidia0:/dev/nvidia0 \
	--device /dev/nvidiactl:/dev/nvidiactl \
	--device /dev/bus/usb:/dev/bus/usb \
	--privileged \
	$@
