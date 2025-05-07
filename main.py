from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Port, Direction, Color
from pybricks.tools import wait
from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.robotics import DriveBase

hub = PrimeHub()
hub.speaker.volume(100)

left_motor: Motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor: Motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)

wheel_diameter: float = 5
axel_track: float = 12.8

base: DriveBase = DriveBase(left_motor, right_motor, wheel_diameter, axel_track)

base.use_gyro(True)

spinning_ultrasonic: UltrasonicSensor = UltrasonicSensor(Port.A)
ultrasonic_motor: Motor = Motor(Port.E)

front_ultrasonic: UltrasonicSensor = UltrasonicSensor(Port.F)

color_sensor: ColorSensor = ColorSensor(Port.B)

class Robot:
    def __init__(self, hub: PrimeHub, base: DriveBase, spinning_ultrasonic: UltrasonicSensor, ultrasonic_motor: Motor, 
                 front_ultrasonic: UltrasonicSensor, color_sensor: ColorSensor) -> None:
        self.hub: PrimeHub = hub
        self.base: DriveBase = base

        self.spinning_ultrasonic: UltrasonicSensor = spinning_ultrasonic
        self.ultrasonic_motor: Motor = ultrasonic_motor

        self.front_ultrasonic: UltrasonicSensor = front_ultrasonic

        self.color_sensor = color_sensor

        self.wall_dist: int = 120

    def check_front_dist(self) -> bool:
        dist = self.front_ultrasonic.distance()

        if dist < self.wall_dist:
            return True

        else:
            return False

    def check_rescue(self) -> bool:
        return (self.color_sensor.color() == Color.RED)

    def loop(self) -> None:
        while True:
            wall: bool = self.check_front_dist()

            if wall:
                base.turn(90)
                continue

            side_dist: int = self.spinning_ultrasonic.distance()
            if side_dist > 2000:
                side_dist = self.wall_dist

            if self.check_rescue():
                self.base.stop()
                self.hub.speaker.beep(duration=5000)
                self.base.straight(50)

            max_turn_val: int = 85
            error = max(min((self.wall_dist - side_dist), max_turn_val), -max_turn_val)
            print(error)

            self.base.drive(20, error)

print("Hello there")

robot = Robot(hub, base, spinning_ultrasonic, ultrasonic_motor, front_ultrasonic, color_sensor)
robot.loop()
hub.speaker.beep(duration=1000)

