#!/bin/sh

USER_UID=$(id -u)
USER_GID=$(id -g)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

USER_ROOT="$(pwd)"

docker run \
	-i \
	-p 8888:8888          \
   	-w /          \
	--privileged=true \
	--net=host \
	--volume=/home/kazu/icub/drake-distro:/drake-distro:rw\
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	--volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw \
	--volume=/dev/bus/usb:/dev/bus/usb:rw \
	--volume=/dev:/dev:rw \
	--env="XAUTHORITY=${XAUTH}" \
	--env="USER_UID=${USER_UID}" \
	--env="USER_GID=${USER_GID}" \
	--env="DISPLAY=${DISPLAY}" \
	-e QT_X11_NO_MITSHM=1\
	--name=drake_kazu1 \
	-t drake bash \

export containerId='docker ps -l -q'

### Notes

# `xhost +local:root`

# To test after installation
# cd /drake-distro
# bazel build //tools:drake_visualizer

# To build everything
# bazel build //...
# bazel test //...

# Examples
# ./bazel-bin/tools/drake_visualizer&
# bazel run //drake/examples/acrobot:acrobot_run_passive
# bazel run //drake/examples/acrobot:acrobot_run_swing_up
# bazel run //drake/examples/contact_model:bowling_ball
# bazel run //drake/examples/kuka_iiwa_arm:kuka_simulation
# bazel run //drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:monolithic_pick_and_place_demo

# Mount other volumes (e.g. drake distro, configs)
