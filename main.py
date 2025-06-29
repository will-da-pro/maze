#!/usr/bin/env python3

import time

from devices.lidar import Lidar

lidar: Lidar = Lidar.init_c3()
lidar.start()
i = 0
while i < 100000:
    print(lidar.pop_buffer())
    i += 1
lidar.stop()
