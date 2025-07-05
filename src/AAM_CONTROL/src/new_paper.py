#!/usr/bin/env python3
import rospy
import math
import numpy as np
from collections import deque
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from ma_rrt_path_plan.msg import WaypointsArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import osqp
from scipy import sparse

# MPC horizon and system dimensions
N = 10
nx = 4   # [X, Y, psi, v]
nu = 2   # [delta, a]
L = 1.5
dt = 0.1  # 10 Hz

# Cost matrices (increased yaw weight for better cornering)
Q = np.diag([5.0, 5.0, 2.0, 0.1])
R = np.diag([0.1, 0.05])

# Actuator limits
delta_max = 0.4
a_max = 2.0

# Globals for waypoints & state
ref_waypoints = []
waypoint_history = deque(maxlen=5)

class VehicleState:
    def __init__(self):
        self.psi = 0.0
        self.v = 0.0

vehicle_state = VehicleState()

# Publishers (initialized in listener)
robot_control_pub = None
pub1 = None  # /v_target
pub2 = None  # /v_actual


def calculate_curvature(x_points, y_points):
    if len(x_points) < 3:
        return 0.0
    dx = np.gradient(x_points)
    dy = np.gradient(y_points)
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)
    curv = np.abs((dx * d2y - dy * d2x) / (dx**2 + dy**2)**(3 / 2))
    return float(np.nanmean(curv)) if curv.size else 0.0


def Another_speed(curv):
    return 6.0 / (1.5 + curv)


def smooth_waypoints(raw_pts):
    waypoint_history.append(raw_pts)
    smoothed = []
    for i in range(len(raw_pts)):
        xs = [h[i][0] for h in waypoint_history if len(h) > i]
        ys = [h[i][1] for h in waypoint_history if len(h) > i]
        smoothed.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return smoothed


def linearized_dynamics(x, delta):
    A = np.eye(4)
    A[0,2] = -x[3] * math.sin(x[2]) * dt
    A[0,3] =  math.cos(x[2]) * dt
    A[1,2] =  x[3] * math.cos(x[2]) * dt
    A[1,3] =  math.sin(x[2]) * dt
    A[2,3] =  math.tan(delta) / L * dt

    B = np.zeros((4,2))
    B[2,0] = x[3] / (L * math.cos(delta)**2) * dt
    B[3,1] = dt

    return A, B


def solve_mpc_osqp(x0, ref_traj):
    P = sparse.block_diag([Q]*N + [R]*N).tocsc()
    q = np.zeros((N*nx + N*nu,))
    for i in range(N):
        q[i*nx:(i+1)*nx] = -Q @ ref_traj[i]

    # dynamics constraints
    Aeq = sparse.lil_matrix((N*nx, N*nx + N*nu))
    beq = np.zeros((N*nx,))
    A, B = linearized_dynamics(x0, 0.0)
    for i in range(N):
        xi = i*nx; ui = N*nx + i*nu
        if i == 0:
            Aeq[xi:xi+nx, xi:xi+nx] = np.eye(nx)
            beq[xi:xi+nx] = x0
        else:
            xip = (i-1)*nx
            Aeq[xi:xi+nx, xi:xi+nx] = np.eye(nx)
            Aeq[xi:xi+nx, xip:xip+nx] = -A
            Aeq[xi:xi+nx, ui:ui+nu] = -B
    Aeq = Aeq.tocsc()

    lb = np.full((N*nx + N*nu,), -np.inf)
    ub = np.full((N*nx + N*nu,),  np.inf)
    for i in range(N):
        ui = N*nx + i*nu
        lb[ui:ui+nu] = [-delta_max, -a_max]
        ub[ui:ui+nu] = [ delta_max,  a_max]

    prob = osqp.OSQP()
    prob.setup(P=P, q=q, A=Aeq, l=beq, u=beq, verbose=False)
    res = prob.solve()
    if res.info.status != 'solved':
        rospy.logwarn("MPC solver failed: %s", res.info.status)
        return np.zeros(nu)
    return res.x[N*nx : N*nx+nu]


def imu_callback(msg: Imu):
    global angvel, vehicle_state
    # 1) extract the Z‐axis angular velocity
    angvel = msg.angular_velocity.z

    # 2) convert quaternion → yaw
    _, _, vehicle_state.psi = euler_from_quaternion([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ])

    # 3) log both yaw and angular rate
    rospy.loginfo("imu_callback: psi=%.3f rad, angvel=%.3f rad/s",
                  vehicle_state.psi, angvel)


def odom_callback(msg: Odometry):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    vehicle_state.v = math.hypot(vx, vy)


def waypoints_callback(wp_msg: WaypointsArray):
    global ref_waypoints

    # Extract raw (x,y) pairs in vehicle frame
    raw = [(wp.x, wp.y) for wp in wp_msg.waypoints]
    if not raw:
        ref_waypoints.clear()
        return

    # Smooth them (they're already in base_link!)
    ref_waypoints = smooth_waypoints(raw)

    rospy.loginfo("Waypoints (local) after smoothing: %d", len(ref_waypoints))


def control_timer_callback(event):
    
    rospy.loginfo("control_timer → %d waypoints", len(ref_waypoints))
    
    if not ref_waypoints:
        rospy.logwarn_throttle(5, "MPC SKIP: no waypoints")
        return

    # curvature & speed
    xs = [pt[0] for pt in ref_waypoints]
    ys = [pt[1] for pt in ref_waypoints]
    k = calculate_curvature(xs, ys)
    if k < 0.02:   max_v = 0.6
    elif k < 0.05: max_v = 0.4
    else:          max_v = 0.25
    v_des = min(Another_speed(k), max_v)

    # build state & reference
    x0 = np.array([0.0, 0.0, vehicle_state.psi, vehicle_state.v])
    ref_traj = []
    for i in range(N):
        x_ref, y_ref = ref_waypoints[min(i, len(ref_waypoints)-1)]
        psi_ref = math.atan2(y_ref, x_ref)  # desired heading for cornering
        ref_traj.append(np.array([x_ref, y_ref, psi_ref, v_des]))

    # solve MPC
    delta, accel = solve_mpc_osqp(x0, ref_traj)
    v_cmd = x0[3] + accel * dt
    v_cmd = max(min(v_cmd, v_des), 0.0)

    # publish command
    cmd = AckermannDriveStamped()
    cmd.drive.steering_angle            = float(delta)
    cmd.drive.speed                     = float(v_cmd)
    cmd.drive.steering_angle_velocity   = 0.0
    cmd.drive.acceleration              = 0.0
    cmd.drive.jerk                      = 0.0
    robot_control_pub.publish(cmd)

    pub1.publish(Float64(data=v_des))
    pub2.publish(Float64(data=vehicle_state.v))

    rospy.logdebug("MPC → k=%.4f, steer=%.3f accel=%.3f v_cmd=%.3f v_act=%.3f", k, delta, accel, v_cmd, vehicle_state.v)


def listener():
    global robot_control_pub, pub1, pub2

    rospy.init_node('control', anonymous=True)
    rospy.Subscriber('/sensor_imu_hector', Imu, imu_callback)
    rospy.Subscriber('/ekf_odom',Odometry, odom_callback)
    rospy.Subscriber('/waypoints', WaypointsArray, waypoints_callback)

    robot_control_pub = rospy.Publisher('/robot_control/command',AckermannDriveStamped,queue_size=0)
    pub1 = rospy.Publisher('/v_target', Float64, queue_size=1)
    pub2 = rospy.Publisher('/v_actual', Float64, queue_size=1)

    rospy.Timer(rospy.Duration(0.1), control_timer_callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
