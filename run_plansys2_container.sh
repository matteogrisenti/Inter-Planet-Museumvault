#!/bin/bash

# 1. Percorso locale sul tuo Mac (dove hai i file PDDL e il codice)
LOCAL_PATH="/Users/dev/Desktop/Automated Planning/project/project/Inter-Planet-Museumvault/2.5"

# 2. Avvio del container
# Mappiamo la tua cartella direttamente dentro la cartella 'src' del workspace nel docker
docker run --platform linux/amd64 \
    -v "$LOCAL_PATH:/plansys2_ws" \
    --network=host \
    --name plansys2_cont \
    --privileged \
    --shm-size 2g \
    --rm \
    -e DISPLAY=host.docker.internal:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -it plansys2 bash