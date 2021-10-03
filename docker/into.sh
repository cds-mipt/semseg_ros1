#!/bin/bash
docker exec --user "docker_semseg" -it semseg \
        /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/docker_semseg; /bin/bash"
