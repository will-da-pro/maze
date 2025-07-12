#!/usr/bin/env python3

from devices.lidar import Lidar

lidar: Lidar = Lidar.init_c3()
lidar.stop()
