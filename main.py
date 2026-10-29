#!/usr/bin/env pybricks-micropython
# Add team number, names, PIDs, and required source citations.

from pybricks import ev3brick as brick
from pybricks.ev3devices import (
    Motor, UltrasonicSensor, TouchSensor, GyroSensor
)
from pybricks.parameters import Port, Button
from pybricks.tools import StopWatch
import math


class MyRobot:
    def __init__(self):
        self.left_motor = Motor(Port.B)
        self.right_motor = Motor(Port.C)

        self.touch_sensor = TouchSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S2)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S3)
        self.touch_sensor_2 = TouchSensor(Port.S4)

        self.display_timer = StopWatch()
        self.action_timer = StopWatch()
        self.motion_timer = StopWatch()
        self.heading_timer = StopWatch()

        self.wheel_radius = 0.027
        self.wheel_separation = 0.118

        self.step = "Waiting"
        self.recovery_mode = "Normal"

        # Local pose: meters and degrees.
        self.x = 0
        self.y = 0
        self.theta = 90

        self.prev_left_angle = self.left_motor.angle()
        self.prev_right_angle = self.right_motor.angle()

        self.retreat_start_left = 0
        self.retreat_start_right = 0
        self.gyro_start_angle = 0

        self.wall_distance_traveled = 0
        self.target_distance = 2.2

        self.motion_left = self.left_motor.angle()
        self.motion_right = self.right_motor.angle()
        self.motion_gyro = self.gyro_sensor.angle()
        self.heading_reference = self.motion_gyro

        self.command_left = 0
        self.command_right = 0

        self.backup_start_left = 0
        self.backup_start_right = 0
        self.recovery_gyro_start = 0

        # Experimental wall-loss threshold, in millimeters.
        self.wall_lost_mm = 500

    def bumper_pressed(self):
        return (
            self.touch_sensor.pressed()
            or self.touch_sensor_2.pressed()
        )

    def drive_following(self, left_speed, right_speed):
        self.command_left = left_speed
        self.command_right = right_speed
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def hold_following(self):
        self.command_left = 0
        self.command_right = 0
        self.left_motor.hold()
        self.right_motor.hold()

    def reset_motion_check(self):
        self.motion_left = self.left_motor.angle()
        self.motion_right = self.right_motor.angle()
        self.motion_gyro = self.gyro_sensor.angle()
        self.motion_timer.reset()

    def reset_heading_check(self):
        self.heading_reference = self.gyro_sensor.angle()
        self.heading_timer.reset()

    def begin_backup(self, mode):
        self.hold_following()
        self.backup_start_left = self.left_motor.angle()
        self.backup_start_right = self.right_motor.angle()
        self.recovery_mode = mode
        self.action_timer.reset()
        self.reset_motion_check()
        self.reset_heading_check()

    def update_position(self):
        left_angle = self.left_motor.angle()
        right_angle = self.right_motor.angle()

        delta_left = math.radians(
            left_angle - self.prev_left_angle
        )
        delta_right = math.radians(
            right_angle - self.prev_right_angle
        )

        distance_left = self.wheel_radius * delta_left
        distance_right = self.wheel_radius * delta_right

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

        # Signed robot travel, not exact distance along the wall.
        if self.step == "Following":
            self.wall_distance_traveled += distance

        self.prev_left_angle = left_angle
        self.prev_right_angle = right_angle

    def follow_wall(self):
        distance_mm = self.ultrasonic_sensor.distance()
        gyro_angle = self.gyro_sensor.angle()
        left_angle = self.left_motor.angle()
        right_angle = self.right_motor.angle()
        contact = self.bumper_pressed()

        if self.wall_distance_traveled >= self.target_distance:
            self.hold_following()
            self.step = "Finished"
            return

        # Requested heading-based recovery.
        # This also triggers during steady straight driving.
        if self.recovery_mode != "Normal":
            self.reset_heading_check()

        elif abs(gyro_angle - self.heading_reference) >= 3:
            self.reset_heading_check()

        elif self.heading_timer.time() >= 8000:
            self.begin_backup("StuckBack")

        # Check for absence of measured wheel/heading movement.
        moving = (
            abs(left_angle - self.motion_left) >= 3
            or abs(right_angle - self.motion_right) >= 3
            or abs(gyro_angle - self.motion_gyro) >= 2
        )

        commanded_to_move = (
            self.command_left != 0
            or self.command_right != 0
        )

        if moving or not commanded_to_move:
            self.reset_motion_check()

        # Do not repeatedly restart an active backup.
        if (
            contact
            and self.recovery_mode not in (
                "BumpBack", "BumpTurn",
                "StuckBack", "Blocked"
            )
        ):
            self.begin_backup("BumpBack")

        elif (
            commanded_to_move
            and self.motion_timer.time() >= 8000
        ):
            if self.recovery_mode in (
                "BumpBack", "BumpTurn", "StuckBack"
            ):
                self.hold_following()
                self.recovery_mode = "Blocked"
            else:
                self.begin_backup("StuckBack")

        if self.recovery_mode in ("BumpBack", "StuckBack"):
            backed_degrees = (
                self.backup_start_left - left_angle
                + self.backup_start_right - right_angle
            ) / 2

            # Approximately 3 cm for the stated wheel radius.
            if backed_degrees >= 64:
                self.hold_following()

                if contact:
                    self.recovery_mode = "Blocked"

                elif self.recovery_mode == "BumpBack":
                    self.recovery_gyro_start = gyro_angle
                    self.recovery_mode = "BumpTurn"
                    self.action_timer.reset()

                else:
                    self.recovery_mode = "TryForward"
                    self.action_timer.reset()

                self.reset_motion_check()

            else:
                self.drive_following(-70, -70)

        elif self.recovery_mode == "BumpTurn":
            turned = abs(
                gyro_angle - self.recovery_gyro_start
            )

            if turned >= 90:
                self.hold_following()
                self.recovery_mode = "TryForward"
                self.action_timer.reset()
                self.reset_motion_check()

            elif self.action_timer.time() >= 6000:
                self.hold_following()
                self.recovery_mode = "Blocked"

            else:
                self.drive_following(70, -70)

        elif self.recovery_mode == "TryForward":
            if self.action_timer.time() >= 500:
                self.hold_following()
                self.recovery_mode = "Normal"
                self.reset_motion_check()
                self.reset_heading_check()
            else:
                self.drive_following(70, 70)

        elif self.recovery_mode == "SearchPause":
            self.hold_following()

            if distance_mm < self.wall_lost_mm:
                self.recovery_mode = "Normal"
                self.reset_heading_check()

            elif self.action_timer.time() >= 200:
                self.recovery_gyro_start = gyro_angle
                self.recovery_mode = "SearchLeft"
                self.action_timer.reset()
                self.reset_motion_check()

        elif self.recovery_mode == "SearchLeft":
            searched_angle = abs(
                gyro_angle - self.recovery_gyro_start
            )

            if distance_mm < self.wall_lost_mm:
                self.hold_following()
                self.recovery_mode = "Normal"
                self.reset_motion_check()
                self.reset_heading_check()

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
            if distance_mm >= self.wall_lost_mm:
                self.hold_following()
                self.recovery_mode = "SearchPause"
                self.action_timer.reset()

            elif distance_mm > 200:
                self.drive_following(80, 110)

            elif distance_mm < 100:
                self.drive_following(110, 80)

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
                        <= self.retreat_start_left - 10
                        or self.right_motor.angle()
                        <= self.retreat_start_right - 10
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

                    if abs(turned_angle) >= 90:
                        self.hold_following()
                        self.wall_distance_traveled = 0
                        self.recovery_mode = "Normal"
                        self.reset_heading_check()
                        self.reset_motion_check()
                        self.step = "Following"
                    else:
                        self.drive_following(90, -90)

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

                elif self.step == "Finished":
                    self.hold_following()
                    brick.sound.beep()
                    break

        finally:
            # Stop motors if a Python exception exits run().
            self.hold_following()


def main():
    lab2 = MyRobot()
    lab2.run()


if __name__ == "__main__":
    main()