# Do not use. Use docker-compose instead.
docker run -it \
--privileged \
--net=host \
-v /dev:/dev/ \
-v /run/udev/:/run/udev/ \
--group-add video \
-e HOME=/tpm \
maze/ros
