#!/bin/bash

xhost +local:docker
docker run -it \
	--rm --net=host --env="DISPLAY" --device=/dev/dri/ \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	robohub:latest