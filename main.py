from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Port, Direction
from pybricks.tools import wait
from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.robotics import DriveBase

hub = PrimeHub()
hub.speaker.volume(50)
hub.speaker.beep(duration=100)

left_motor: Motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor: Motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)

wheel_diameter: float = 5
axel_track: float = 12.8

base: DriveBase = DriveBase(left_motor, right_motor, wheel_diameter, axel_track)

base.use_gyro(True)

base.turn(360)
base.drive(100, 1)

spinning_ultrasonic: UltrasonicSensor = UltrasonicSensor(Port.A)
ultrasonic_motor: Motor = Motor(Port.E)

front_ultrasonic: UltrasonicSensor = UltrasonicSensor(Port.B)

class Robot:
    def __init__(self, hub: PrimeHub, base: DriveBase, spinning_ultrasonic: UltrasonicSensor, ultrasonic_motor: Motor, front_ultrasonic: UltrasonicSensor) -> None:
        self.hub: PrimeHub = hub
        self.base: DriveBase = base

        self.spinning_ultrasonic: UltrasonicSensor = spinning_ultrasonic
        self.ultrasonic_motor: Motor = ultrasonic_motor

        self.front_ultrasonic: UltrasonicSensor = front_ultrasonic

        self.wall_dist: int = 200

    def check_front_dist(self) -> bool:
        dist = self.front_ultrasonic.distance()

        if dist < self.wall_dist:
            return True

        else:
            return False

    def loop(self) -> None:
        while True:
            wall: bool = self.check_front_dist()

            if wall:
                return

            self.base.drive(100, 0)

robot = Robot(hub, base, spinning_ultrasonic, ultrasonic_motor, front_ultrasonic)
robot.loop()
hub.speaker.beep(duration=1000)

