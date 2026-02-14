#!/bin/bash
docker run --platform linux/amd64 -v '/Users/dev/Desktop/Automated Planning/project:/project' --network=host --name ubuntu_bash --privileged --shm-size 2g --rm -i -t myplanutils bash