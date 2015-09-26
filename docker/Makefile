all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all cogrob images"
	# @echo "   1. make pull             - pull all cogrob images"
	# @echo "   1. make clean            - remove all cogrob images"
	@echo ""

build:
		@docker build --tag=ros:indigo-cuda cuda/.
		@docker build --tag=ros:indigo-caffe caffe/.
		@docker build --tag=ros:indigo-ros-caffe ros-caffe/.
		@docker build --tag=ros:indigo-ros-caffe-dev dev/.

# pull:
# 	@docker pull ros

# clean:
# 	@docker rmi -f ros
