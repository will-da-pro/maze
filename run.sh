docker run -it \
--privileged \
--net=host \
-v /dev:/dev/ \
-v /run/udev/:/run/udev/ \
--group-add video \
-e HOME=/tpm \
maze/ros \
ros2 launch maze_navigator maze.launch.py
