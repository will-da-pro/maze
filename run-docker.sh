docker run -it \
--privileged \
--net=host \
-v /dev:/dev/ \
-v /run/udev/:/run/udev/ \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--group-add video \
-e HOME=/tpm \
-e DISPLAY=$DISPLAY \
maze/ros
