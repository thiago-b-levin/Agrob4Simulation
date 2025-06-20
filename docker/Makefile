UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))/../
SRC_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))/../webots_packages/
NVIDIA_GPU:=$(shell docker info | grep Runtimes | grep nvidia 1> /dev/null && nvidia-smi 1> /dev/null && echo '--gpus all' || echo '')
DOCKERUSER:=cyberbotics
BASENAME:=webots_ros2
DOCKERUSERCOMMENT:="Cyberbotics"
ROS_DISTRO?=humble
ROS_TESTING?=0
WEBOTS_VERSION?=2025a

build:
	echo "ROS_TESTING=${ROS_TESTING}"
	docker build ${ROOT_DIR} \
		-f ${ROOT_DIR}/docker/Dockerfile \
		-t ${BASENAME}-image ${DOCKER_ARGS} \
		--build-arg UID=${UID} \
		--build-arg DOCKERUSER=${DOCKERUSER} \
		--build-arg DOCKERUSERCOMMENT=${DOCKERUSERCOMMENT} \
		--build-arg ROS_TESTING=${ROS_TESTING} \
		--build-arg WEBOTS_VERSION=${WEBOTS_VERSION} \
		--build-arg ROS_DISTRO=${ROS_DISTRO} \
		--build-arg NVIDIA_DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | cut -d '.' -f1)

run:
	@docker container rm -f ${BASENAME}-container
	docker run \
		--net=host \
		--ipc=host \
		--cap-add SYS_ADMIN \
		--name ${BASENAME}-container \
		--privileged \
		--restart unless-stopped \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e __NV_PRIME_RENDER_OFFLOAD=1 \
		-e __GLX_VENDOR_LIBRARY_NAME=nvidia \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/${DOCKERUSER}/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro \
		-v ${SRC_DIR}:/${DOCKERUSER}/ros2_ws/src/ \
		-v /dev:/dev:rw \
		--entrypoint bash \
		-d -it ${BASENAME}-image


exec:
	docker exec -it ${BASENAME}-container bash
