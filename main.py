#!/usr/bin/env python3

import numpy as np

from devices.lidar import Lidar
from math import sin, cos, radians

class Landmark:
    def __init__(self, life: int) -> None:
        self.x_pos: int | None = None
        self.y_pos: int | None = None

        self.total_times_observed: int = 0
        self.id: int | None = None

        self.life: int = life

        self.a: float | None = None
        self.b: float | None = None

        self.distance: float | None = None
        self.distance_error: float | None = None

        self.bearing: float | None = None
        self.bearing_error: float | None = None


class Landmarks:
    MAX_LANDMARKS: int       = 3000
    MAX_ERROR: float         = 0.5
    MIN_OBSERVATIONS: int    = 15
    LIFE: int                = 40
    MAX_RANGE: int           = 1
    FAILURE_THRESHOLD: float = 8.1

    # RANSAC Parameters
    MAX_TRIALS: int          = 1000
    MAX_SAMPLE: int          = 10
    MIN_LINE_POINTS: int     = 30
    RANSAC_TOLERANCE: float  = 0.05
    RANSAC_CONSENSUS: int    = 30

    def __init__(self, degrees_per_scan: float) -> None:
        self.id_to_id: np.ndarray = np.zeros((self.MAX_LANDMARKS, 2), dtype=int)
        self.ekf_landmarks: int = 0

        self.landmark_db: np.ndarray = np.full(self.MAX_LANDMARKS, Landmark(self.LIFE), dtype=Landmark)
        self.db_size: int = 0

        self.degrees_per_scan: float = degrees_per_scan

    def get_slam_id(self, id: int) -> int:
        for row in self.id_to_id:
            if row[0] == id:
                return row[1]

        return -1

    def add_slam_id(self, landmark_id: int, slam_id: int) -> None:
        self.id_to_id[self.ekf_landmarks][0] = landmark_id
        self.id_to_id[self.ekf_landmarks][1] = slam_id

        self.ekf_landmarks += 1

    def remove_bad_landmarks(self, laser_data: dict[float, float], robot_position: tuple[float, float], robot_angle: float) -> None:
        max_range: float = 0

        distances: list[float] = list(laser_data.values())
        for index, distance in enumerate(distances[1:len(laser_data)]):
            if (distances[index - 1] > self.FAILURE_THRESHOLD or distances[index + 1] > self.FAILURE_THRESHOLD):
                continue

            if distance > max_range:
                max_range = distance

        max_range = self.MAX_RANGE

        x_bounds: np.ndarray = np.zeros(4, dtype=float)
        y_bounds: np.ndarray = np.zeros(4, dtype=float)

        x_bounds[0] = cos(radians(self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE + robot_position[0]
        y_bounds[0] = sin(radians(self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE + robot_position[1]

        x_bounds[1] = x_bounds[0] + cos(radians(180 * self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE
        y_bounds[1] = y_bounds[0] + sin(radians(180 * self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE
        
        x_bounds[2] = cos(radians(359 * self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE + robot_position[0]
        y_bounds[2] = sin(radians(359 * self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE + robot_position[1]

        x_bounds[3] = x_bounds[2] + cos(radians(180 * self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE
        y_bounds[3] = y_bounds[2] + sin(radians(180 * self.degrees_per_scan) + radians(robot_angle)) * self.MAX_RANGE

        for index, landmark in enumerate(self.landmark_db):
            pnt_x: float = landmark.x_pos
            pnt_y: float = landmark.y_pos


class SLAM:
    def __init__(self, min_x: int = -1000, max_x: int = 1000, min_y: int = -1000, max_y: int = 1000) -> None:
        self.min_x: int = min_x
        self.max_x: int = max_x
        self.min_y: int = min_y
        self.max_y: int = max_y

        self.rows: int = self.max_y - self.min_y + 1
        self.cols: int = self.max_x - self.min_x + 1

        self.grid: np.ndarray = np.full((self.rows, self.cols), -1, dtype=int)

        self.current_x: float = 0
        self.current_y: float = 0

    def to_physical(self, logical_x: int, logical_y: int) -> tuple[int, int]:
        physical_x: int = logical_x - self.min_x
        physical_y: int = logical_y - self.min_y

        return physical_x, physical_y

    def to_logical(self, physical_x: int, physical_y: int) -> tuple[int, int]:
        logical_x: int = physical_x + self.min_x
        logical_y: int = physical_y + self.min_y

        return logical_x, logical_y


class Robot:
    def __init__(self) -> None:
        pass


lidar: Lidar = Lidar.init_c3()
lidar.start()
i = 0
while i < 100000:
    print(lidar.pop_buffer())
    i += 1
lidar.stop()
