#!/usr/bin/env pybricks-micropython
# Seungyoun Lee, PID: 730902524
# Maria Caballero, PID: 731063735

from pybricks import ev3brick as brick
from pybricks.ev3devices import (
    Motor, UltrasonicSensor, TouchSensor, GyroSensor
)
from pybricks.parameters import Port, Button
from pybricks.tools import StopWatch, wait
import math


class MyRobot:
    def __init__(self):
        self.left_motor = Motor(Port.B)
        self.right_motor = Motor(Port.C)

        # Left front bumper: S1. Right front bumper: S4.
        self.touch_sensor = TouchSensor(Port.S1)
        self.touch_sensor_2 = TouchSensor(Port.S4)

        self.gyro_sensor = GyroSensor(Port.S2)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S3)

        self.display_timer = StopWatch()
        self.action_timer = StopWatch()

        self.wheel_radius = 0.027
        self.wheel_separation = 0.118

        self.step = "Waiting"
        self.recovery_mode = "Normal"

        self.x = 0
        self.y = 0
        self.theta = 90

        self.prev_left_angle = self.left_motor.angle()
        self.prev_right_angle = self.right_motor.angle()

        self.retreat_start_left = 0
        self.retreat_start_right = 0
        self.gyro_start_angle = 0

        # Estimated robot travel during following, in meters.
        self.wall_distance_traveled = 0
        self.target_distance = 2.2

        self.recovery_gyro_start = 0
        self.wall_lost_mm = 500

        # Approximately 5 cm of backward wheel travel.
        self.backup_start_left = 0
        self.backup_start_right = 0
        self.backup_degrees = math.degrees(
            0.05 / self.wheel_radius
        )
        

    def bumper_pressed(self):
        return (
            self.touch_sensor.pressed()
            or self.touch_sensor_2.pressed()
        )

    def drive_following(self, left_speed, right_speed):
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def hold_following(self):
        self.left_motor.hold()
        self.right_motor.hold()

    def update_position(self):
        left_angle = self.left_motor.angle()
        right_angle = self.right_motor.angle()

        distance_left = self.wheel_radius * math.radians(
            left_angle - self.prev_left_angle
        )
        distance_right = self.wheel_radius * math.radians(
            right_angle - self.prev_right_angle
        )

        distance = (distance_left + distance_right) / 2
        delta_theta = (
            distance_right - distance_left
        ) / self.wheel_separation

        midpoint_theta = (
            math.radians(self.theta) + delta_theta / 2
        )

        self.x += distance * math.cos(midpoint_theta)
        self.y += distance * math.sin(midpoint_theta)
        self.theta += math.degrees(delta_theta)

        if self.step == "Following":
            # Backing up subtracts from estimated progress.
            self.wall_distance_traveled += distance

        self.prev_left_angle = left_angle
        self.prev_right_angle = right_angle

    def follow_wall(self):
        distance_mm = self.ultrasonic_sensor.distance()
        gyro_angle = self.gyro_sensor.angle()

        if self.wall_distance_traveled >= self.target_distance:
            self.hold_following()
            self.step = "Finished"
            return

        # Left bumper triggers a 5 cm backup during normal following.
        if (
            self.recovery_mode == "Normal"
            and self.touch_sensor.pressed()
        ):
            self.hold_following()
            self.backup_start_left = self.left_motor.angle()
            self.backup_start_right = self.right_motor.angle()
            self.recovery_mode = "LeftBack"
            self.action_timer.reset()

        if self.recovery_mode == "LeftBack":
            left_back = (
                self.backup_start_left - self.left_motor.angle()
            )
            right_back = (
                self.backup_start_right - self.right_motor.angle()
            )

            if (
                left_back >= self.backup_degrees
                and right_back >= self.backup_degrees
            ):
                self.hold_following()

                if self.bumper_pressed():
                    self.recovery_mode = "Blocked"
                else:
                    self.recovery_gyro_start = gyro_angle
                    self.recovery_mode = "LeftTurn"
                    self.action_timer.reset()

            elif self.action_timer.time() >= 5000:
                self.hold_following()
                self.recovery_mode = "Blocked"

            else:
                self.drive_following(-200, -200)

        elif self.recovery_mode == "LeftTurn":
            turned = abs(
                gyro_angle - self.recovery_gyro_start
            )

            # Check contact before allowing forward movement again.
            if self.bumper_pressed():
                self.hold_following()
                self.recovery_mode = "Blocked"

            elif turned >= 20:
                self.hold_following()
                self.recovery_mode = "Normal"

            elif self.action_timer.time() >= 4000:
                self.hold_following()
                self.recovery_mode = "Blocked"

            else:
                self.drive_following(80, -80)

        elif self.recovery_mode == "SearchPause":
            self.hold_following()

            if 0 <= distance_mm < self.wall_lost_mm:
                self.recovery_mode = "Normal"

            elif self.action_timer.time() >= 200:
                self.recovery_gyro_start = gyro_angle
                self.recovery_mode = "SearchLeft"
                self.action_timer.reset()

        elif self.recovery_mode == "SearchLeft":
            searched_angle = abs(
                gyro_angle - self.recovery_gyro_start
            )

            if 0 <= distance_mm < self.wall_lost_mm:
                self.hold_following()
                self.recovery_mode = "Normal"

            elif (
                searched_angle >= 60
                or self.action_timer.time() >= 5000
            ):
                self.hold_following()
                self.recovery_mode = "Blocked"

            else:
                self.drive_following(-40, 40)

        elif self.recovery_mode == "Blocked":
            self.hold_following()

        elif self.recovery_mode == "Normal":
            if distance_mm < 0 or distance_mm >= self.wall_lost_mm:
                self.hold_following()
                self.recovery_mode = "SearchPause"
                self.action_timer.reset()

            elif distance_mm > 150:
                # Wall on left: steer toward it.
                self.drive_following(80, 110)

            elif distance_mm < 100:
                # Wall on left: steer away from it.
                self.drive_following(120, 50)

            else:
                self.drive_following(100, 100)

        if self.display_timer.time() >= 150:
            brick.display.clear()
            brick.display.text(
                "Mode: " + self.recovery_mode
            )
            brick.display.text(
                "Wall cm: " + str(distance_mm / 10)
            )
            brick.display.text(
                "Gyro: " + str(gyro_angle)
            )
            brick.display.text(
                "Travel m: "
                + str(round(self.wall_distance_traveled, 3))
            )
            self.display_timer.reset()

    def run(self):
        try:
            while True:
                self.update_position()

                # DOWN ends the program and holds both motors.
                if Button.DOWN in brick.buttons():
                    break

                if self.step == "Waiting":
                    self.hold_following()

                    if Button.CENTER in brick.buttons():
                        self.step = "Approach"

                elif self.step == "Approach":
                    if self.bumper_pressed():
                        self.hold_following()
                        self.retreat_start_left = (
                            self.left_motor.angle()
                        )
                        self.retreat_start_right = (
                            self.right_motor.angle()
                        )
                        self.step = "Retreat"
                    else:
                        self.drive_following(270, 270)

                elif self.step == "Retreat":
                    if (
                        self.left_motor.angle()
                        <= self.retreat_start_left - 150
                        or self.right_motor.angle()
                        <= self.retreat_start_right - 150
                    ):
                        self.hold_following()
                        self.gyro_start_angle = (
                            self.gyro_sensor.angle()
                        )
                        self.step = "Rotate"
                    else:
                        self.drive_following(-50, -50)

                elif self.step == "Rotate":
                    current_angle = self.gyro_sensor.angle()
                    turned_angle = (
                        current_angle - self.gyro_start_angle
                    )

                    if abs(turned_angle) >= 80:
                        self.hold_following()

                        # Start measuring wheel travel here.
                        self.prev_left_angle = self.left_motor.angle()
                        self.prev_right_angle = self.right_motor.angle()

                        self.wall_distance_traveled = 0
                        self.recovery_mode = "Normal"
                        self.step = "Following"
                    else:
                        self.drive_following(80, -80)

                    if self.display_timer.time() >= 150:
                        brick.display.clear()
                        brick.display.text(
                            "Sensor: " + str(current_angle)
                        )
                        brick.display.text(
                            "Start: " + str(self.gyro_start_angle)
                        )
                        brick.display.text(
                            "Turned: " + str(turned_angle)
                        )
                        self.display_timer.reset()

                elif self.step == "Following":
                    self.follow_wall()

                    if self.recovery_mode == "Blocked":
                        self.hold_following()
                        brick.display.clear()
                        brick.display.text("Blocked: stopped")
                        break

                elif self.step == "Finished":
                    self.hold_following()
                    brick.sound.beep()
                    break

                wait(20)

        finally:
            self.hold_following()


def main():
    lab2 = MyRobot()
    lab2.run()


if __name__ == "__main__":
    main()