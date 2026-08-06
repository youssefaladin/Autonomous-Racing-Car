#!/usr/bin/env python3
"""
Lateral controller for the FSD acceleration mission.

Implements an extended Stanley controller. The steering command is the sum of
four terms:

    1. heading error against the reference path
    2. a softened cross-track correction, atan(k * e / (k_soft + v))
    3. yaw-rate damping
    4. a speed-dependent understeer feedforward, -kss * v^2 * curvature

The run is terminated by detecting the orange cones that mark the end of the
acceleration straight.
"""
import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion

# Stanley gains
K_CROSS_TRACK = 0.003125   # cross-track error gain
K_SOFT = 3.0               # softening constant, keeps the term finite at low speed
K_YAW_DAMPING = 0.0159375  # yaw-rate damping gain
K_UNDERSTEER = 120 / (2 * 10000)  # understeer gradient for the curvature feedforward

WHEELBASE = 1.53           # m, used for the yaw-rate estimate
LATERAL_OFFSET = 0.7675    # m, camera-to-centreline offset on the target point
CRUISE_SPEED = 2.0         # m/s commanded during the run
STOP_DISTANCE = 4.7        # m, distance to the orange cones that ends the run

# Vehicle state
steering_ang = 0.0
v_act = 0.0
k = 0.0001                 # path curvature estimate
yaw = 0.0
angvel = 0.0

# Latched once the end-of-run cones are reached, so the car stays stopped.
run_finished = False

# PID state, kept for the speed controller below
prev_error = 0.0
integral = 0.0

robot_control_pub = None


def pid(current_speed , desired_speed):
        global integral
        global prev_error 
        error = desired_speed - current_speed
        integral = integral + error
        derivative = error - prev_error
        control_output = 0.1 * error + 0.01 * integral + 0.01 * derivative
        prev_error = error
        return control_output

def Another_speed(curv):
        v = 3/(1+curv)
        return v


def calculate_curvature(x_points,y_points):
            
            dx = np.gradient(x_points)
            dy = np.gradient(y_points)
            d2x = np.gradient(dx)
            d2y = np.gradient(dy)
                
            curvature = np.abs((dx * d2y - dy * d2x) / (dx**2 + dy**2)**(3 / 2))

            return np.mean(curvature)

def calculate_distance(x_points,y_points,i):
        try:
                x_point = x_points[i]
                y_point = y_points[i]
        except IndexError:
                x_point = x_points[0]
                y_point = y_points[0]

        return math.hypot(x_point, y_point)


def publish_stop():
        """Command zero speed and zero steering."""
        stop = AckermannDriveStamped()
        stop.drive.steering_angle = 0
        stop.drive.speed = 0
        stop.drive.steering_angle_velocity = 0
        stop.drive.acceleration = 0
        stop.drive.jerk = 0
        robot_control_pub.publish(stop)

        



def waypoints_callback(wp):
        """Compute the Stanley steering command for the next waypoint."""
        global steering_ang

        if run_finished:
                publish_stop()
                return

        x_points = []
        y_points = []
        for point in wp.markers[0].points:
                x_points.append(point.x)
                y_points.append(point.y)

        if not x_points:
                return

        x_target = x_points[0] + LATERAL_OFFSET
        y_target = y_points[0]

        # Heading error, measured against the line from the car to the target point.
        heading_angle_ref = math.atan2(y_target, x_target)

        # Cross-track error: lateral offset of the target point from the heading line.
        ec = y_target
        yaw_rate = (v_act * math.sin(steering_ang)) / WHEELBASE

        steering_ang = (
                # heading error, less the steady-state understeer feedforward
                (heading_angle_ref - K_UNDERSTEER * v_act ** 2 * k)
                # softened cross-track correction
                + math.atan((K_CROSS_TRACK * ec) / (K_SOFT + v_act))
                # yaw-rate damping
                + K_YAW_DAMPING * (yaw_rate - v_act * k)
        )

        rospy.logdebug("steering=%.4f rad  v=%.2f m/s", steering_ang, v_act)

        command = AckermannDriveStamped()
        command.drive.steering_angle = steering_ang
        command.drive.speed = CRUISE_SPEED
        command.drive.steering_angle_velocity = 0
        command.drive.acceleration = 0
        command.drive.jerk = 0
        robot_control_pub.publish(command)


def imu_callback(data):
        global angvel, yaw
        angvel = data.angular_velocity.z
        orientation_list = [data.orientation.x, data.orientation.y,
                            data.orientation.z, data.orientation.w]
        (_roll, _pitch, yaw) = euler_from_quaternion(orientation_list)


def odom_callback(odom):
        global v_act
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v_act = math.hypot(vx, vy)
        rospy.logdebug("v_act=%.2f m/s", v_act)


def cones_callback(cone):
        """Detect the orange cones that mark the end of the acceleration run."""
        global run_finished

        if run_finished:
                return

        cones_yellow = []
        cones_blue = []
        cones_orange = []
        big_cone = []
        for cone_marker in cone.markers:
                cone_x = cone_marker.pose.position.x
                cone_y = cone_marker.pose.position.y
                colour = cone_marker.color
                if colour.r == 0 and colour.g == 0 and colour.b == 200:
                        cones_blue.append((cone_x, cone_y))
                elif colour.r == 200 and colour.g == 200 and colour.b == 0:
                        cones_yellow.append((cone_x, cone_y))
                elif colour.r == 200 and colour.g == 100 and colour.b == 0:
                        cones_orange.append((cone_x, cone_y))
                elif colour.r == 200 and colour.g == 0 and colour.b == 0:
                        big_cone.append((cone_x, cone_y))

        # The run is over only when the track cones are gone and orange remains.
        if cones_yellow or cones_blue or big_cone or not cones_orange:
                return

        x_orange, y_orange = cones_orange[0]
        distance = math.hypot(x_orange, y_orange)

        if distance < STOP_DISTANCE:
                rospy.loginfo("End of acceleration run at %.2f m, stopping.", distance)
                run_finished = True
                publish_stop()




def listener():
        global robot_control_pub
        global v_target_pub
        global v_actual_pub

        rospy.init_node('stanley_acceleration', anonymous=True)

        robot_control_pub = rospy.Publisher(
                "/robot_control/command", AckermannDriveStamped, queue_size=1)
        v_target_pub = rospy.Publisher("/v_target", Float64, queue_size=1)
        v_actual_pub = rospy.Publisher("/v_actual", Float64, queue_size=1)

        rospy.Subscriber('/visual/waypoints', MarkerArray, waypoints_callback)
        rospy.Subscriber('/sensor_imu_hector', Imu, imu_callback)
        rospy.Subscriber("/ground_truth/state_raw", Odometry, odom_callback)
        rospy.Subscriber('/camera_cones_marker', MarkerArray, cones_callback)


if __name__ == "__main__":
        listener()
        rospy.spin()
