#!/usr/bin/env python3

""" This script uses the gpiozero library to access the gpio pins on the Raspberry Pi. The speed of the motors are varied to allow maneuverability of the robot in any direction. More information about the gpiozero library can be found here at https://gpiozero.readthedocs.io/en/stable/api_pins.html?highlight=PiGpioFactory#gpiozero.pins.pigpio.PiGPIOFactory """

# ROS
import rospy
from geometry_msgs.msg import Twist

# gpiozero
from gpiozero import Robot
from gpiozero.pins.pigpio import PiGPIOFactory

# Set up the pin_factory to get better control of the GPIO pins.
FACTORY = PiGPIOFactory()

# Types of motion.
CURVILINEAR = "curvilinear motion"
LINEAR = "linear motion"
ROTATIONAL = "rotational motion"
NO_MOTION = "no motion"


class MotorControl:
    """This class allows the robot to navigate in any direction by varying the speeds of the individual motors."""

    def __init__(self) -> None:

        # Set up the right & left motors by initializing them as tuples. Pin numbers are based on the specific expanision board schematic. More info can be found here http://www.yahboom.net/study/4wd-ban
        self.left_motor_pins = (20, 21, 16)
        self.right_motor_pins = (19, 26, 13)
        self.m1_motors = Robot(
            left=self.left_motor_pins,
            right=self.right_motor_pins,
            pin_factory=FACTORY,
        )

        # Subscriber
        self.vel_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self.control_motor_speed_cb)

        # Initialize the linear and angular velocities to zero.
        self.linear: float = 0.0  # m/s
        self.angular: float = 0.0  # rad/s

        # Scaling constants for amount of curvature in motion.
        self.curve_scale: float = 0.6  # rad/s

    def type_of_motion(self) -> str:
        """ Determine the type of motion. Motion can either be curvilinear, linear, rotational or no motion depending on the linear and angular velocities. """

        # Both linear and angular velocities are non-zero.
        if self.linear and self.angular:
            return CURVILINEAR

        # Only linear velocity is non-zero.
        elif self.linear and not self.angular:
            return LINEAR

        # Only angular velocity is non-zero.
        elif self.angular and not self.linear:
            return ROTATIONAL

        return NO_MOTION  # Both linear or angular velocities are zero.

    def control_motor_speed_cb(self, vel) -> None:
        """Callback which handles the logic that controls the speed of the motors. The speed of each motor is determined by the robot motion. """

        # TODO: Set a limit on the max speed for safety reasons.
        # Linear vel, m/s
        self.linear = vel.linear.x if vel.linear.x else 0.0
        # Angular vel, rad/s
        self.angular = vel.angular.z if vel.angular.z else 0.0

        # Determine the type of motion.
        motion_type = self.type_of_motion()

        # Robot motion is along a curved path.
        if motion_type is CURVILINEAR:

            # Curve forwards towards the right (frame of robot)
            if self.linear > 0.0 and self.angular < 0.0:
                self.m1_motors.forward(
                    speed=abs(self.angular), curve_right=self.curve_scale)

            # Curve forwards towards the left (frame of robot)
            elif self.linear > 0.0 and self.angular > 0.0:
                self.m1_motors.forward(
                    speed=self.angular, curve_left=self.curve_scale)

            # Curve backwards towards the right (frame of robot)
            elif self.linear < 0.0 and self.angular > 0.0:
                self.m1_motors.backward(
                    speed=self.angular, curve_right=self.curve_scale)

            # Curve backwards towards the left (frame of robot)
            elif self.linear < 0.0 and self.angular < 0.0:
                self.m1_motors.backward(
                    speed=abs(self.angular), curve_left=self.curve_scale)

        # Drive robot linearly either forwards or backwards.
        elif motion_type is LINEAR:
            if self.linear > 0:
                self.m1_motors.forward(speed=self.linear)
            else:
                self.m1_motors.backward(speed=abs(self.linear))

        # Rotate robot about z-axis.
        elif motion_type is ROTATIONAL:
            if self.angular > 0:
                self.m1_motors.left(speed=self.angular)
            else:
                self.m1_motors.right(speed=abs(self.angular))

        # Stop the robot.
        else:
            self.m1_motors.stop()

        # Log info about they type of motion, linear vel and angular vel.
        msg = f"Robot has linear velocity of: [{self.linear}] m/s and an angular velocity of: [{self.angular}] rad/s."
        rospy.loginfo(msg)


if __name__ == "__main__":
    try:
        # Initialize ROS node.
        rospy.init_node("motor_control_node")
        MotorControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        error_msg = "Cannot communicate with ROS MASTER. Shutting down motors."
        rospy.logerr(error_msg)
        FACTORY.close()  # Close the pin factory.
    if KeyboardInterrupt:
        warn_msg = "Motors shutting down due to a keyboard interrupt."
        rospy.logwarn(warn_msg)
        FACTORY.close()  # Close the pin factory.
