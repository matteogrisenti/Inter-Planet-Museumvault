#!/bin/bash

LOCAL_PROJECT_DIR="/Users/dev/Desktop/Automated Planning/project/project/Inter-Planet-Museumvault/2.5:/plansys2_ws"

docker run --platform linux/amd64 \
    -v "$LOCAL_PROJECT_DIR" \
    --network=host \
    --name plansys2_cont \
    --privileged \
    --shm-size 2g \
    --rm \
    -e DISPLAY=host.docker.internal:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -i -t plansys2 bash