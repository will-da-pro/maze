from serial import Serial
import time
import pygame
from math import sin, cos, pi

class Lidar:
    def __init__(self, port: str, baudrate: int, timeout: int = 1) -> None:
        self.port: str = port
        self.baudrate: int = baudrate
        self.timeout: int = timeout

        self.ser: Serial = Serial(self.port, self.baudrate, timeout = self.timeout)

    def start(self) -> None:
        self.ser.write(b'\xA5\x20')

        descriptor: bytes = b'\xA5\x5A\x05\x00\x00\x40\x81'

        if self.ser.read(7) != descriptor:
            print("Lidar Not Started!")
            self.stop()

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

    def stop(self) -> None:
        self.ser.write(b'\xA5\x25')

    def read(self) -> bytes:
        buffer: bytes = self.ser.read(5)

        return buffer

    def __del__(self) -> None:
        self.ser.close()

if __name__ == '__main__':
    pygame.init()
    surface = pygame.display.set_mode((400, 400))

    ser = Lidar("/dev/ttyUSB0", 460800, timeout=1)
    ser.start()

    start_time = time.time()
    t = 60


    while True:
        try:
            raw = ser.read()

            if len(raw) < 5:
                continue

            try:
                new_scan, quality, angle, distance = ser.process_scan(raw)
            except Exception as e:
                print(e)
                continue

            if quality == 0:
                continue

            x = distance * sin(angle * pi / 180) / 10 + 200
            y = distance * cos(angle * pi / 180) / 10 + 200

            print(f"Quality: {quality}, Angle: {angle}, Distance: {distance}, X: {x}, Y: {y}")

            if new_scan:
                pygame.display.update()
                surface.fill((0, 0, 0))

            pygame.draw.circle(surface, (255, 0, 0), (x, y), 3)

            if time.time() > start_time + t:
                break


        except Exception as e:
            print(e)
            break

    ser.stop()
   
