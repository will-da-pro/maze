#!/usr/bin/env python3

import math
import signal
import time

from serial import Serial
from serial.tools import list_ports
from threading import Thread

class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x: float = x
        self.y: float = y

    def __eq__(self, value: object, /) -> bool:
        if not isinstance(value, Point):
            return False

        elif self.x == value.x and self.y == value.y:
            return True

        else:
            return False


class LaserPoint:
    def __init__(self, angle: float, distance: float) -> None:
        self.angle: float = angle
        self.distance: float = distance

    def to_laser_point(self, robot_position: Point) -> Point:
        x: float = math.sin(math.radians(self.angle)) * self.distance + robot_position.x
        y: float = math.cos(math.radians(self.angle)) * self.distance + robot_position.y

        return Point(x, y)


class Lidar:
    START_FLAG: bytes                = b'\xA5'
    START_FLAG_2: bytes              = b'\x5A'

    REQUEST_STOP: bytes              = b'\x25'
    REQUEST_RESET: bytes             = b'\x40'
    REQUEST_SCAN: bytes              = b'\x20'
    REQUEST_EXPRESS_SCAN: bytes      = b'\x86'
    REQUEST_FORCE_SCAN: bytes        = b'\x21'
    REQUEST_GET_INFO: bytes          = b'\x50'
    REQUEST_GET_HEALTH: bytes        = b'\x52'
    REQUEST_GET_SAMPLERATE: bytes    = b'\x59'

    DESCRIPTOR_SCAN: bytes           = b'\x05\x00\x00\x40\x81'
    DESCRIPTOR_EXPRESS_SCAN: bytes   = b'\x54\x00\x00\x40\x82'
    DESCRIPTOR_FORCE_SCAN: bytes     = b'\x05\x00\x00\x40\x81'
    DESCRIPTOR_GET_HEALTH: bytes     = b'\x03\x00\x00\x00\x06'
    DESCRIPTOR_GET_SAMPLERATE: bytes = b'\x04\x00\x00\x00\x15'

    PRODUCT_ID: int                  = 0xEA60
    VENDOR_ID: int                   = 0x10C4

    protection_stops: int = 0
    active: bool = False
    health: int = -1

    health_strs: dict[int, str] = {
        -1: "Unknown",
        0: "OK",
        1: "Warning",
        2: "Hardware Failure"
    }

    def __init__(self, port: str, baudrate: int, timeout: int = 1, buffer_size: int = 20) -> None:
        self.port: str = port
        self.baudrate: int = baudrate
        self.timeout: int = timeout

        self.ser: Serial = Serial(self.port, self.baudrate, timeout = self.timeout)

        self.frame_buffer: list[list[LaserPoint]] = []
        self.buffer_size: int = buffer_size
        self.buffer_thread: Thread = Thread(target=self.update_buffer)

        signal.signal(signal.SIGINT, signal.SIG_DFL)

    @classmethod
    def init_c3(cls):
        found_port: str | None = None
        for port in list_ports.comports():
            if port.vid == cls.VENDOR_ID and port.pid == cls.PRODUCT_ID:
                found_port = port.device
                print(f"Found lidar at port {found_port}")
                break

        if found_port is None:
            raise Exception("Lidar not found!")

        return cls(found_port, 460800)

    def check_health(self) -> int:
        self.stop()
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.write(self.START_FLAG + self.REQUEST_GET_HEALTH)
        time.sleep(0.05)

        descriptor = self.ser.read(7)

        if descriptor != self.START_FLAG + self.START_FLAG_2 + self.DESCRIPTOR_GET_HEALTH:
            return 1

        raw: bytes = self.ser.read(3)

        status: int = raw[0]
        error_code: int = raw[1] + raw[2] << 8

        if status == 0:
            print("Health OK")

        elif status == 1:
            print("Warning: Possible Hardware Failure!")
            print(error_code)
        else:
            print("Error: Hardware Failure!")

            if self.protection_stops == 0:
                self.send_reset()
                status = self.check_health()
            
            else:
                raise Exception(error_code)
        
        self.health = status
        return status

    def send_reset(self) -> None:
        self.ser.write(self.START_FLAG + self.REQUEST_RESET)

        self.protection_stops += 1

        time.sleep(0.002)

    def start(self) -> None:
        health = self.check_health()

        if health != 0:
            return

        self.ser.write(self.START_FLAG + self.REQUEST_SCAN)

        self.active = True

        if self.ser.read(7) != self.START_FLAG + self.START_FLAG_2 + self.DESCRIPTOR_SCAN:
            print("Lidar Not Started!")
            self.stop()
            self.active = False

        self.buffer_thread.start()

        print("Scanning started successfully")

    def process_scan(self, raw: bytes) -> tuple[bool, int, float, float]:
        new_scan: bool = bool(raw[0] & 0b1)
        inversed_new_scan: bool = bool((raw[0] >> 1) & 0b1)
        quality: int = raw[0] >> 2

        if new_scan == inversed_new_scan:
            self.ser.reset_input_buffer()
            raise Exception('New scan flags mismatch')

        check_bit: int = raw[1] & 0b1

        if check_bit != 1:
            self.ser.reset_input_buffer()
            raise Exception('Check bit not equal to 1')

        angle: float = ((raw[1] >> 1) + (raw[2] << 7)) / 64
        distance: float = (raw[3] + (raw[4] << 8)) / 4

        return new_scan, quality, angle, distance

    def update_buffer(self) -> None:
        while True:
            try:
                if not self.active:
                    break

                raw = self.read()

                if len(raw) < 5:
                    continue

                try:
                    new_scan, quality, angle, distance = self.process_scan(raw)

                except Exception as e:
                    print(e)
                    continue

                if new_scan:
                    self.frame_buffer.append([])

                if len(self.frame_buffer) > self.buffer_size:
                    self.frame_buffer.pop(0)

                if quality < 5:
                    continue

                if len(self.frame_buffer) > 0:
                    self.frame_buffer[len(self.frame_buffer) - 1].append(LaserPoint(angle, distance))

            except Exception as e:
                print(e)
                return

    def pop_buffer(self) -> list[LaserPoint]:
        while len(self.frame_buffer) < 2:
            if not self.active:
                return []

            time.sleep(0.005)

        buffer: list[LaserPoint] = self.frame_buffer.pop(len(self.frame_buffer) - 2)
        return buffer

    def stop(self) -> None:
        self.ser.write(self.START_FLAG + self.REQUEST_STOP)
        self.active = False

        print("Stopped Scanning")

    def read(self) -> bytes:
        buffer: bytes = self.ser.read(5)

        return buffer

    def __del__(self) -> None:
        if self.active:
            self.stop()

        self.ser.close()

if __name__ == '__main__':
    import pygame

    from math import sin, cos, pi

    pygame.init()
    pygame.font.init()
    surface = pygame.display.set_mode((800, 800))
    font_size: int = 30
    font = pygame.font.SysFont("Times New Roman", 30)

    ser = Lidar.init_c3()
    ser.start()

    start_time = time.time()
    t = 300

    size = 800
    scale = 0.1

    while True:
        try:
            if not ser.active:
                break

            raw = ser.read()

            if len(raw) < 5:
                continue

            try:
                new_scan, quality, angle, distance = ser.process_scan(raw)
            except Exception as e:
                print(e)
                continue

            x = distance * sin(angle * pi / 180) * scale + size / 2
            y = -distance * cos(angle * pi / 180) * scale + size / 2

            #print(f"Quality: {quality}, Angle: {angle}, Distance: {distance}, X: {x}, Y: {y}")

            if new_scan:
                pygame.draw.circle(surface, (0, 0, 255), (size / 2, size / 2), 5)

                time_text = font.render(f"Time: {time.time() - start_time}", False, (0, 255, 0))
                health_text = font.render(f"Health: {ser.health_strs[ser.health]}", False, (0, 255, 0))

                surface.blit(time_text, (5, 5))
                surface.blit(health_text, (5, font_size + 5))

                pygame.display.update()
                surface.fill((0, 0, 0))

            if quality == 0:
                continue
            
            pygame.draw.line(surface, (255, 255, 255), (size / 2, size / 2), (x, y), 2)
            pygame.draw.circle(surface, (255, 0, 0), (x, y), 3)

            if time.time() > start_time + t:
                break


        except Exception as e:
            print(e)
            break

    ser.stop()
   
