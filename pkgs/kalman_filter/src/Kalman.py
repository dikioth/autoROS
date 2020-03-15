'''
The kalman filter.
Author: Elvis Rodas.
'''
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from filterpy.kalman import KalmanFilter
from tf.transformations import euler_from_quaternion


class Kalman:
    '''
        The Kalman Filter
        To build the kalman filter it is important to provide the measurement values and the variance of respective
        state variable.
        Data in standard units. [meters,secs, etc]

        Equation used is:
                            x = F * x  + B * u + w      ----- (1)
                            z = H * x + v               ----- (2)

        TODO: Explain the kalman filter algorithm.

        Args:
            odom (Odometry msg) : The odometry data containing the Pose (position, orientation and measurement covariance
                                    of respective variable) and Twist (Linear and angluar velocity with covariance).
                                    More info: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html

            imu (Imu msg)       : The Imu data containing the angular and linear acceleration with the measurement covariance.
                                    More info: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html

            process_var         : Process variance. A dictionary holding the diagonal elements of the 8x8 process covariance matrix.
                                  format: { process_cov_xpos: ...,
                                            process_cov_ypos: ...,
                                            process_cov_xvel: ...,
                                            process_cov_yvel: ...,
                                            process_cov_xacc: ...,
                                            process_cov_yacc: ...,
                                            process_cov_yaw : ...,
                                            process_cov_yawvel: .. }

            only_uwb (Boolean)  : jUse only location data. This is important to construct dimention of state space.

            OBS: The measurement covariance matrix R will be built from the covariance stored in 'odom' and 'imu'.
                  It is expected that the ROS msgs have those values.

        Attributes:
            kf: The kalman Filter object created from filterpy.kalman module.

        Public Methods:
            kalman_predic:  perform predicition.
            kalman_update: perform update.

        Private Methods:
            setX    : Sets the state vector from eq. (1)-(2)
            setF    : Sets the process matrix F matrix from eq. (1)
            setB    : Sets the control matrix B from eq. (1)
            setH    : Sets the measurement matrix H from eq. (2)
            setQ    : Sets the process covariance matrix Q needed for the kalman filter.
            setR    : Sets the measruements covariance matrix R needed for the kalman filter.

    '''

    def __init__(self, odom, imu, dt, only_uwb=False):
        # TODO: Explanation 'c'
        c = 0.9  # Speed divided by length.

        self.only_uwb = only_uwb

        # Update dimentions. u - control vector. x - state space vars. z - measurement vector.
        if self.only_uwb:
            dim_u = 0   # No control.
            dim_x = 4   # [x, y, yaw, x_vel, y_vel]
            dim_z = 2   # [x, y, -  ,  -   ,   -  ]
        else:
            dim_u = 1   # yaw control.
            dim_x = 8   # [x, y, yaw, yaw_vel, x_vel, y_vel, x_accel, y_accel]
            dim_z = 5   # [x, y, yaw,   -    ,  -   , -    , x_accel, y_accel]

        # initialize Kalman filter
        self.kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z, dim_u=dim_u)

        # init state vector and matrices.
        self.kf.x = self.__setX(odom.pose.pose.position)
        self.kf.F = self.__setF(dt)
        self.kf.B = self.__setB(c)
        self.kf.H = self.__setH()

        # Filter variables
        self.kf.P *= 10
        # measurement noise covariance
        self.kf.R = self.__setR(odom.pose.covariance, odom.twist.covariance,
                                imu.angular_velocity_covariance, imu.linear_acceleration_covariance)
        # Diagonal elements of process covariance matrix.
        # TODO: let user input the process covariance matrix Q.
        defaultQ = np.array([.0, .0, .0, .08, .1, .1,  .08, .08])
        self.kf.Q = self.__setQ(defaultQ)  # process noise
        rospy.loginfo("KF initiated")

    def __setX(self, position):
        '''
            setX. Private method initiates the state space.

            Args:
                position: A geometry_msgs/Point msg containing the x,y,z position. Only x,y will be used.

            Returns:
                X: a nx1 vector containing the state space  [x, y, x_vel, y_vel]^T
                   velocity is set to zero (assuming car at rest at time 0).

        '''

        X = np.zeros([4, 1])
        X[0, 0] = position.x
        X[1, 0] = position.y

        return X

    def __setF(self, dt):
        '''
            Args:
                dt: The time step.
            Returns:
                F: 4x4 The state transition matrix.
        '''

        F = np.array([[1., 0., dt, 0.],
                      [0., 1., 0., dt],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]])

        return F

    def __setB(self, dt):
        '''
            Args:
                dt : The time step.
            Returns:
                B : The 4x2 control matrix.
        '''

        B = np.array([[0.5*dt*dt, 0],
                      [0,         0.5*dt*dt],
                      [dt,        0],
                      [0,         dt]])

        return B

    def __setH(self):
        '''
        setH: Sets the control matrix.
            returns:
                    The 2x4 control matrix. 
        '''
        H = np.array([[1., 0., 0., 0.],
                      [0., 1., 0., 0.]])

        return H

    def __setQ(self):
        '''
        setQ: Private method that computes the 8x8 process covariance matrix Q.

            Args:
                processQ: The process covariance matrix. Expecting row vector.
                          TODO: If arg is a 1xn vector, assume diagonal elements are given, else a square matrix must be given.
            Returns:
                Q: The process noise covariance matrix.

        '''

        Q = np.diag(np.array([0.1, 0.1, 0.01, 0.01]))

        return Q

    def __setR(self, pos_var, vel_var, angular_accel_var, linear_accel_var):
        '''
        SetR: Private method. Computes measurement noise matrix R.
            Args:
                pos_var             : 6x6 diagonal matrix. Position (x,y,z) variance.
                                      OBS: This matrix include also the variance of the orientation (raw, pitch, yaw),
                                      because of the structure of the 'Pose' ROS msg. Only the variance of x,y will be used here.
                                      (See http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html)

                vel_var             : 6x6 diagonal matrix. Velocity (vx,vy,vz) variance.
                angular_accel_var   : 3x3 diagonal matrix. Angular acceleration (a_roll, a_pitch, a_yaw) variance.
                linear_accel_var    : 3x3 diagonal matrix. Linear acceleration (ax, ay, az).

            Returns:
                The measurement covariance matrix R.
        '''
        R = np.diag(np.array([0.1, 0.1]))
        return R

    def kalman_predic(self, imu):
        '''
        predic. Public method. Performs the first step of KF.
        Arguments:
                The IMU image. Only the linear acceleration will be used as the control input.
        '''

        ctrl = np.array([[imu.linear_acceleration.x],
                         [imu.linear_acceleration.y]])

        self.kf.predict(u=ctrl)

    def kalman_update(self, odom):
        '''
        Update. Public method. Performs the second step of KF.

        Args:
            position: A Pose message (geometry_msgs/Pose.msg)
            imu     : An Imu message (sensor_msgs/Imu.msg)
        Returns:
            None

        '''

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        self.kf.update(np.array([[x], [y]]))

    def get_estimation(self):
        return self.kf.x
