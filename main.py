#!/usr/bin/env python3

import math
import numpy as np
import pygame

from devices.lidar import Lidar

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

        x_bounds[0] = math.cos(math.radians(self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE + robot_position[0]
        y_bounds[0] = math.sin(math.radians(self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE + robot_position[1]

        x_bounds[1] = x_bounds[0] + math.cos(math.radians(180 * self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE
        y_bounds[1] = y_bounds[0] + math.sin(math.radians(180 * self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE
        
        x_bounds[2] = math.cos(math.radians(359 * self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE + robot_position[0]
        y_bounds[2] = math.sin(math.radians(359 * self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE + robot_position[1]

        x_bounds[3] = x_bounds[2] + math.cos(math.radians(180 * self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE
        y_bounds[3] = y_bounds[2] + math.sin(math.radians(180 * self.degrees_per_scan) + math.radians(robot_angle)) * self.MAX_RANGE

        for index, landmark in enumerate(self.landmark_db):
            pnt_x: float = landmark.x_pos
            pnt_y: float = landmark.y_pos

            print(pnt_x, pnt_y)


class FeaturesDetection:
    def __init__(self) -> None:
        pass


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

        self.angle_bounds: float = 10

    def to_physical(self, logical_x: int, logical_y: int) -> tuple[int, int]:
        physical_x: int = logical_x - self.min_x
        physical_y: int = logical_y - self.min_y

        return physical_x, physical_y

    def to_logical(self, physical_x: int, physical_y: int) -> tuple[int, int]:
        logical_x: int = physical_x + self.min_x
        logical_y: int = physical_y + self.min_y

        return logical_x, logical_y

    def get_angle_between_points(self, left_point: tuple[float, float], midpoint: tuple[float, float], right_point: tuple[float, float]) -> float:
        A: tuple[float, float] = math.sin(math.radians(left_point[0])) * left_point[1], math.cos(math.radians(left_point[0])) * left_point[1]
        B: tuple[float, float] = math.sin(math.radians(midpoint[0])) * midpoint[1], math.cos(math.radians(midpoint[0])) * midpoint[1]
        C: tuple[float, float] = math.sin(math.radians(right_point[0])) * right_point[1], math.cos(math.radians(right_point[0])) * right_point[1]

        BA: tuple[float, float] = A[0] - B[0], A[1] - B[1]
        BC: tuple[float, float] = C[0] - B[0], C[1] - B[1]

        denom: float = math.sqrt(BA[0] ** 2 + BA[1] ** 2) * math.sqrt(BC[0] ** 2 + BC[1] ** 2)

        if denom == 0:
            return 0

        return math.acos((BA[0] * BC[0] + BA[1] * BC[1]) / denom)

    def eliminate_possible_corners(self, scan: list[tuple[float, float]]) -> list[tuple[float, float]]:
        scan_copy: list[tuple[float, float]] = scan.copy()
        min_length: int = 15
        last_angle: int = -1

        for index, point in enumerate(scan):
            if index == len(scan) - 1:
                angle: float = self.get_angle_between_points(scan[index - 1], point, scan[0])

            else:
                angle: float = self.get_angle_between_points(scan[index - 1], point, scan[index + 1])

            if angle < math.radians(180 - self.angle_bounds) or angle > math.radians(180 + self.angle_bounds):
                scan_copy.remove(point)

                if 1 < index - last_angle and index - last_angle < min_length:
                    for point2 in scan[last_angle + 1:index - 1]:
                        scan_copy.remove(point2)

                last_angle = index

        print("final", len(scan_copy))
        print("old", len(scan))
        return scan_copy

    def find_lines(self, scan: list[tuple[float, float]]):
        scan = self.eliminate_possible_corners(scan)


class Robot:
    def __init__(self) -> None:
        pass


lidar: Lidar = Lidar.init_c3()
lidar.start()
slam: SLAM = SLAM()


pygame.display.init()
surface = pygame.display.set_mode((800, 800))
size: int = 800


i = 0
while i < 100000:
    graph: np.ndarray = np.zeros((size, size), dtype=np.uint8)

    lidar_data: list[tuple[float, float]] = lidar.pop_buffer()
    new_data: list[tuple[float, float]] = slam.eliminate_possible_corners(lidar_data)

    surface.fill((0, 0, 0))
    for point in new_data:
        x: int = int(1 * math.sin(math.radians(point[0])) * point[1]) + size // 2
        y: int = -int(1 * math.cos(math.radians(point[0])) * point[1]) + size // 2 
        pygame.draw.line(surface, (255, 255, 255), (size / 2, size / 2), (x, y), 2)
        pygame.draw.circle(surface, (255, 0, 0), (x, y), 3)
    pygame.display.update() 

    i += 1
lidar.stop()
