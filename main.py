#!/usr/bin/env python3

import math
import numpy as np
import pygame

from devices.lidar import Lidar, LaserPoint, Point
from fractions import Fraction
from scipy import odr, stats

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

            #print(pnt_x, pnt_y)


class FeaturesDetection:
    def __init__(self) -> None:
        self.EPSILON: int = 15
        self.DELTA: int = 10
        self.SNUM: int = 6
        self.PMIN: int = 10
        self.GMAX: int = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS: list[tuple[Point, Point]] = []
        self.LASERPOINTS: list[tuple[Point, float]] = []
        self.LINE_PARAMS: tuple[float, float, float] | None = None
        self.NP: int = len(self.LASERPOINTS) - 1
        self.LMIN: int = 2
        self.LR: int = 0
        self.PR: int = 0

    def dist_point2point(self, point1: Point, point2: Point) -> float:
        Px = (point1.x - point2.x) ** 2
        Py = (point1.y - point2.y) ** 2

        return math.sqrt(Px + Py)

    def dist_point2line(self, params: tuple[float, float, float], point: Point) -> float:
        A, B, C = params

        distance: float = abs(A * point.x + B * point.y + C) / math.sqrt(A ** 2 + B ** 2)
        return distance

    def line_2points(self, m: float, b: float) -> list[tuple[float, float]]:
        x: float = 5
        y: float = m * x + b
        x2: float = 2000
        y2: float = m * x2 + b
        return [(x, y), (x2, y2)]

    def lineForm_G2Si(self, A: float, B: float, C: float) -> tuple[float, float]:
        m: float = -A / B
        b: float = -C / B
        return m, b

    def lineForm_Si2G(self, m, b) -> tuple[float, float, float]:
        A, B, C = -m, 1, -b

        if A < 0:
            A, B, C = -A, -B, -C

        den_a: float = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c: float = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd: float = np.gcd(den_a, den_c)
        lcm: float = den_a * den_c / gcd

        A = A * lcm
        B = B * lcm
        C = C * lcm

        return A, B, C

    def line_intersect_general(self, params1: tuple[float, float, float], params2: tuple[float, float, float]) -> Point:
        a1, b1, c1 = params1
        a2, b2, c2 = params2

        x: float = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y: float = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)

        return Point(x, y)

    def points_2line(self, point1: Point, point2: Point) -> tuple[float, float]:
        m: float = 0
        b: float = 0

        if point1.x != point2.x:
            m = (point2.y - point1.y) / (point2.x - point1.x)
            b = point2.y - m * point2.x

        return m, b

    def projection_point2line(self, point: Point, m: float, b: float) -> Point:
        m2: float = -1 / m
        c2: float = point.y - m2 * point.x

        x: float = - (b - c2) / (m - m2)
        y: float = m2 * x + c2

        return Point(x, y)

    def laser_points_set(self, data: list[LaserPoint], robot_position: Point):
        self.LASERPOINTS = []

        if not data:
            pass

        for point in data:
            coordinates: Point = point.to_laser_point(robot_position)
            self.LASERPOINTS.append((coordinates, point.angle))

        self.NP = len(self.LASERPOINTS) - 1

    def linear_func(self, p, x) -> float:
        m, b = p
        return m * x * b

    def odr_fit(self, laser_points: list[Point]) -> tuple[float, float]:
        x: np.ndarray = np.array([point.x for point in laser_points])
        y: np.ndarray = np.array([point.y for point in laser_points])

        #print("Model X:", x)
        #print("Model Y:", y)

        #linear_model: odr.Model = odr.Model(self.linear_func)

        #data: odr.RealData = odr.RealData(x, y, sx=1, sy=1/np.var(y))

        #odr_model: odr.ODR = odr.ODR(data, linear_model, beta0=[0., 0.])

        #out: odr.Output = odr_model.run()
        #print(out.stopreason)
        #m, b = out.beta
        #print(out.beta)

        #m, b = np.polyfit(x, y, 1)
        m, b, r, p, err = stats.linregress(x, y)

        return m, b

    def predict_point(self, line_params: tuple[float, float, float], sensed_point: Point, robot_position: Point) -> Point:
        m, b = self.points_2line(robot_position, sensed_point)
        params1: tuple[float, float, float] = self.lineForm_Si2G(m, b)
        pred: Point = self.line_intersect_general(params1, line_params)

        return pred

    def seed_segment_detection(self, robot_position: Point, break_point_ind: int) -> tuple[list[tuple[Point, float]], list[Point], tuple[int, int]] | None:
        flag: bool = True

        self.NP = max(0, self.NP)
        self.SEED_SEGMENTS = []

        for i in range(break_point_ind, (self.NP - self.PMIN)):
            predicted_points_to_draw: list[Point] = []

            j: int = i + self.SNUM
            m, c = self.odr_fit(list(i[0] for i in self.LASERPOINTS)[i:j])

            params: tuple[float, float, float] = self.lineForm_Si2G(m, c)

            for k in range(i, j):
                predicted_point: Point = self.predict_point(params, self.LASERPOINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)

                d1: float = self.dist_point2point(predicted_point, self.LASERPOINTS[k][0])

                if d1 > self.DELTA:
                    flag = False
                    break

                d2: float = self.dist_point2line(params, predicted_point)
                
                if d2 > self.EPSILON:
                    flag = False
                    break

            if flag:
                self.LINE_PARAMS = params
                return self.LASERPOINTS[i:j], predicted_points_to_draw, (i, j)

    def seed_segment_growing(self, indices: tuple[int, int], break_point: int) -> list | None:
        line_eq: tuple[float, float, float] | None = self.LINE_PARAMS

        if line_eq is None:
            return

        i, j = indices
        PB, PF = max(break_point, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        while self.dist_point2line(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PF > self.NP - 1:
                break

            else:
                m, b = self.odr_fit([i[0] for i in self.LASERPOINTS][PB:PF])
                #print("1)", m, b)
                #print([str(i[0]) for i in self.LASERPOINTS][PB:PF])
                line_eq = self.lineForm_Si2G(m, b)

            POINT = self.LASERPOINTS[PF][0]

            PF += 1
            NEXTPOINT: Point = self.LASERPOINTS[PF][0]

            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PF -= 1

        while self.dist_point2line(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PB < break_point:
                break
            
            else:
                m, b = self.odr_fit([i[0] for i in self.LASERPOINTS][PB:PF])
                #print("2)", m, b)
                #print([str(i[0]) for i in self.LASERPOINTS][PB:PF])
                line_eq = self.lineForm_Si2G(m, b)
            
            POINT: Point = self.LASERPOINTS[PB][0]

            PB -= 1

            NEXTPOINT: Point = self.LASERPOINTS[PB][0]

            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PB += 1

        LR: float = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR: int = len(self.LASERPOINTS[PB:PF])

        #print(LR, PR)

        if LR >= self.LMIN and PR >= self.PMIN:
            #print("Good grow")
            self.LINE_PARAMS = line_eq
            m, b = self.lineForm_G2Si(line_eq[0], line_eq[1], line_eq[2])
            two_points: list[tuple[float, float]] = self.line_2points(m, b)
            self.LINE_SEGMENTS.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))

            #print("Line Equation:", line_eq)

            return [self.LASERPOINTS[PB:PF], two_points, self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0], PF, line_eq, (m, b)]

    
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


pygame.init()
surface = pygame.display.set_mode((800, 800))
size: int = 800
scale: float = 0.2

running: bool = True

while running:
    graph: np.ndarray = np.zeros((size, size), dtype=np.uint8)

    lidar_data: list[LaserPoint] = lidar.pop_buffer()
    featureMAP.laser_points_set(lidar_data, Point(0, 0))

    ENDPOINTS: list[Point | int] = [0, 0]
    surface.fill((0, 0, 0))

    while BREAK_POINT_IND < (featureMAP.NP - featureMAP.PMIN):
        seedSeg = featureMAP.seed_segment_detection(Point(0, 0), BREAK_POINT_IND)
        #print(BREAK_POINT_IND, seedSeg)

        if seedSeg is None:
            BREAK_POINT_IND += 10

        else:
            seedSegment, PREDICTED_POINTS_TODRAW, INDICES = seedSeg
            results = featureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
            #print("Results: ", results)

            if results is None:
                BREAK_POINT_IND = INDICES[1]
                continue

            else:
                line_eq: tuple[float, float] = results[5]
                m, c = results[6]
                line_seg: list[tuple[Point, float]] = results[0]
                OUTERMOST = (results[2], results[3])
                BREAK_POINT_IND = results[4]

                #print(line_eq, m, c, [(str(i[0]), i[1]) for i in line_seg], OUTERMOST, BREAK_POINT_IND)

                ENDPOINTS[0] = featureMAP.projection_point2line(OUTERMOST[0], m, c)
                ENDPOINTS[1] = featureMAP.projection_point2line(OUTERMOST[1], m, c)

                r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)

                for point in line_seg:
                    if math.isnan(point[0].x) or math.isnan(point[0].y):
                        continue

                    pygame.draw.circle(surface, (r, g, b), (int(point[0].x * scale) + size / 2, int(point[0].y * scale) + size / 2), 3)
                    pygame.draw.line(surface, (255, 255, 255), (size / 2, size / 2), (int(point[0].x * scale) + size / 2, int(point[0].y * scale) + size / 2), 2)


                #print("Endpoints:", ENDPOINTS[0], ENDPOINTS[1])

                if math.isnan(ENDPOINTS[0].x) or math.isnan(ENDPOINTS[0].y) or math.isnan(ENDPOINTS[1].x) or math.isnan(ENDPOINTS[1].y):
                    continue

                pygame.draw.line(surface, (0, 255, 0), (int(ENDPOINTS[0].x * scale + size / 2), int(ENDPOINTS[0].y) * scale + size / 2), (int(ENDPOINTS[1].x * scale + size / 2), int(ENDPOINTS[1].y * scale + size / 2)), 2)

    pygame.display.update() 

lidar.stop()
