#!/bin/bash

docker build -f Dockerfile -t robohub ./

# remove intermediate dangling containers
dangling_images=$(docker images -f "dangling=true" -q)
if [ ! -z "$dangling_images" ]
then
	docker rmi -f "$dangling_images"
fi