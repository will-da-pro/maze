#!/usr/bin/env python3

import math
import numpy as np
import pygame

from devices.lidar import Lidar, LaserPoint, Point

class Robot:
    def __init__(self) -> None:
        pass


lidar: Lidar = Lidar.init_c3()
lidar.start()


pygame.init()
surface = pygame.display.set_mode((800, 800))
size: int = 800
scale: float = 0.2

running: bool = True

while running:
    graph: np.ndarray = np.zeros((size, size), dtype=np.uint8)

    lidar_data: list[LaserPoint] = lidar.pop_buffer()

lidar.stop()
