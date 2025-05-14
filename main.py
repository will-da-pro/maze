from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Port, Direction, Color
from pybricks.pupdevices import Motor, UltrasonicSensor, ColorSensor
from pybricks.robotics import DriveBase

from umath import sin, pi

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

print(ultrasonic_motor.angle())

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

        self.current_angle: int = -60

        self.wall_dist: int = 120
        self.side_dist: int = int(self.wall_dist / (sin(abs(self.current_angle) * pi / 180)))

        self.set_ultrasonic(self.current_angle)

    def set_ultrasonic(self, angle: int = -45) -> None:
        centre_angle: int = 90
        current_angle: int = self.ultrasonic_motor.angle()

        target_angle: int = centre_angle + angle

        turn_angle: int = target_angle - current_angle

        self.ultrasonic_motor.run_angle(50, turn_angle)
        self.current_angle = angle
        self.side_dist: int = int(self.wall_dist / (sin(abs(self.current_angle) * pi / 180)))


    def check_front_dist(self) -> bool:
        dist = self.front_ultrasonic.distance()

        if dist < self.wall_dist:
            return True

        else:
            return False

    def check_color(self) -> None:
        if self.color_sensor.color() == Color.RED:
            self.hub.light.on(Color.RED)
            self.hub.speaker.beep(duration=1000)
            self.hub.light.off()
            
            while self.front_ultrasonic.distance() < self.wall_dist:
                self.base.turn(90)

            self.base.straight(100)

        elif self.color_sensor.color() == Color.GREEN:
            self.hub.light.on(Color.GREEN)
            self.hub.speaker.beep(duration=1000)
            self.hub.light.off()
            
            while self.front_ultrasonic.distance() < self.wall_dist:
                self.base.turn(90)

            self.base.straight(100)


    def loop(self) -> None:
        while True:
            wall: bool = self.check_front_dist()

            if wall:
                base.turn(90)
                continue

            side_dist: int = self.spinning_ultrasonic.distance()
            if side_dist > 2000:
                side_dist = self.side_dist

            self.check_color()
                
            max_turn_val: int = 85
            error = max(min((self.side_dist - side_dist), max_turn_val), -max_turn_val)
            print(error)

            self.base.drive(20, error)

print("Hello there")

robot = Robot(hub, base, spinning_ultrasonic, ultrasonic_motor, front_ultrasonic, color_sensor)
robot.loop()
hub.speaker.beep(duration=1000)

