import numpy as np
from filterpy.kalman import KalmanFilter
from nav_msgs.msg import Odometry
# from custom_msgs import LocationData, EstimatedState, IMUData


# var_x and var_y in meters

class Kalman:
    '''
        The Kalman Filter

        Args:
            odom (Odometry msg): The odometry data containing the Pose (position and orientation) 
                             and Twist (Linear and angluar velocity).

        Attributes:
            arg (str): This is where we store arg,
    '''

    def __init__(self,
                 odom,
                 dt,
                 process_var_acc,
                 process_var_vel,
                 process_var_heading,
                 process_var_heading_acc,
                 process_var_pos,
                 meas_var_pos,
                 meas_var_heading,
                 meas_var_acc,
                 speed_div_by_length,
                 use_acc,
                 dim_x,
                 dim_z,
                 dim_u):

        # initialize Kalman filter
        self.kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z, dim_u=dim_u)

        # init state vector x
        self.kf.X = self.__setX(odom.pose.pose.position, use_acc=use_acc)
        self.kf.F = self.__setF(dt, use_acc=use_acc)
        self.kf.H = self.__setH(use_acc=use_acc)
        self.kf.B = self.__setB(c=speed_div_by_length, use_acc=use_acc)

        self.kf.P *= 10

        self.kf.R = self.__setR(var_acc=meas_var_acc, var_position=meas_var_pos,
                                var_heading=meas_var_heading, use_acc=use_acc)  # measurement noise
        self.kf.Q = self.__setQ(dt, var_x=process_var_pos, var_y=process_var_pos, var_acc=process_var_acc,
                                var_heading=process_var_heading, var_heading_acc=process_var_heading_acc,
                                var_velocity=process_var_vel, use_acc=use_acc)  # process noise

    def __setX(self, position, use_acc=False):
        '''
            Args:
                position: A geometry_msgs/Point msg containing the x,y,z position.
            Returns:
                X: a 8x1 matrix containing the state space [x,y,theta, theta_dot, xdot, ydot, xddot, yddot]^T
        '''

        if use_acc:
            x = np.zeros([8, 1])
            x[0, 0] = position.x
            x[1, 0] = position.y
        else:
            # initialize with first position x and y, 0 in v and a
            x = np.array([[position.x], [position.y], [0.0], [0.0]])
        return x

    def __setF(self, dt, use_acc=False):
        '''
            Args:
                dt: The time step.
            Returns:
                F: The process matrix.
        '''
        if use_acc:  # dim_x = 6, dim_z = 4
            F = np.array([[1., 0., 0., 0,  dt, 0., 0.5 * dt * dt, 0.],
                          [0., 1., 0., 0., 0., dt, 0., 0.5 * dt * dt],
                          [0., 0., 1., dt, 0., 0., 0., 0.],
                          [0., 0., 0., 1., 0., 0., 0., 0.],
                          [0., 0., 0., 0., 1., 0., dt, 0.],
                          [0., 0., 0., 0., 0., 1., 0., dt],
                          [0., 0., 0., 0., 0., 0., 1., 0.],
                          [0., 0., 0., 0., 0., 0., 0., 1.]]
                         )
        else:
            F = np.array([[1., 0., dt, 0],
                          [0., 1., 0, dt],
                          [0., 0., 1., 0.],
                          [0., 0., 0., 1.]])

        return F

    def __setH(self, use_acc=False):
        if use_acc:
            H = np.array([[1., 0., 0., 0., 0., 0., 0., 0.],
                          [0., 1., 0., 0., 0., 0., 0., 0.],
                          [0., 0., 1., 0., 0., 0., 0., 0.],
                          [0., 0., 0., 0., 0., 0., 1., 0.],
                          [0., 0., 0., 0., 0., 0., 0., 1.]])
        else:
            H = np.array([[1., 0., 0., 0.],
                          [0., 1., 0., 0.]])
        return H

    def __setQ(self, dt, pose_cov, twist_cov,  var_heading, var_heading_acc, var_velocity, use_acc=False, var_x=0.0, var_y=0.0, var_acc=0.5):
        '''
        setQ: Private method that computes the measurement covariance matrix Q.

        The variance of the position (x,y,z) is substracted from the ROS msg: geometry_msgs/PoseWithCovariance.
        The variance of the position (-,-,yaw) is substracted from the ROS msg: geometry_msgs/PoseWithCovariance

        The variance of the angular velocity, orientation and linear acceleration is extracted from the ROS msg: sensor_msgs/Imu.msg.


            Args:
                pose_cov: A 6x6 diagonal matrix containing the variance of the position (x,y,z) and orientation (roll, pitch, yaw).
                          OBS: only x,y will be used since it is assumed that the robot moves in 2D. The entire 6x6 matrix is sent to be
                          compatible with the ROS message geometry_msgs/PoseWithCovariance.msg.

                twist_cov: A 6x6 diagonal matrix containing the variance of the Twist (3x3 linear and 3x3 angular velocity).
                          OBS: only x,y will be used since it is assumed that the robot moves in 2D. The entire 6x6 matrix is sent to be
                          compatible with the ROS message geometry_msgs/PoseWithCovariance.msg.

            Returns:
                Q: The measurement noise covariance matrix. 

        '''
        if use_acc:
            var_acc_x = var_acc
            var_acc_y = var_acc

            q = np.array([[var_x, 0., 0., 0., 0., 0., 0., 0.],
                          [0., var_y, 0., 0., 0., 0., 0., 0.],
                          [0., 0., var_heading, 0., 0., 0., 0., 0.],
                          [0., 0., 0., var_heading_acc, 0., 0., 0., 0.],
                          [0., 0., 0., 0., var_velocity, 0., 0., 0.],
                          [0., 0., 0., 0., 0., var_velocity, 0., 0.],
                          [0., 0., 0., 0., 0., 0., var_acc_x, 0.],
                          [0., 0., 0., 0., 0., 0., 0., var_acc_y]
                          ])
        else:  # remove this if acc is good
            var_x_dot = var_velocity
            var_y_dot = var_velocity
            q = np.array([[var_x, 0., 0., 0.],
                          [0., var_y, 0., 0.],
                          [0., 0., var_x_dot, 0.],
                          [0., 0., 0., var_y_dot]])
        return q

    def __setB(self, c, use_acc=False):
        '''
            Args:
                c : The speed divided by length v/L according to Ackerman equation.
            Returns:
                B : The B matrix.
        '''
        if use_acc:
            B = np.zeros([8, 1])
            B[3, 0] = c
        else:
            B = 0
        return B

    def __measurement_update(self, position, imu_data, use_acc=False):
        if use_acc:
            z = np.array(
                [[position.x],
                 [position.y],
                 [imu_data.rotation.yaw],
                 [imu_data.linear_acceleration.x],
                 [imu_data.linear_acceleration.y]]
            )
        else:
            z = [[position.x],
                 [position.y]]
        return z

    # TODO: call this function set_R ?
    def __setR(self, var_position, var_acc, var_heading, use_acc=False):
        var_x = var_position
        var_y = var_position

        if use_acc:
            var_acc_y = var_acc
            var_acc_x = var_acc
            r = np.array([[var_x, 0., 0., 0., 0.],
                          [0., var_y, 0., 0., 0.],
                          [0., 0., var_heading, 0., 0.],
                          [0., 0., 0., var_acc_x, 0.],
                          [0., 0., 0., 0., var_acc_y]])
        else:
            r = np.array([[var_x, 0.],
                          [0., var_y]])
        return r

    ####################################################################################################
    # kalman_updates: performs the prediction and update of the Kalman filter
    # If the position is None we keep the last z, but we make the covariance matrix for the measurements
    # have ~infinity numbers, so the prediction is vastly favored over the measurement.
    # Returns: a location estimate as a LocationData
    def kalman_updates(self,
                       position,
                       imu_data,
                       timestep,
                       variance_position,
                       variance_acceleration,
                       variance_heading,
                       process_var_heading,
                       process_var_pos,
                       process_var_acc,
                       process_var_heading_acc,
                       process_var_velocity,
                       u=None,
                       use_acc=True):

        self.kf.F = set_F(timestep, use_acc=use_acc)
        self.kf.Q = set_Q(timestep, var_x=process_var_pos, var_y=process_var_pos,
                          var_acc=process_var_acc, var_heading=process_var_heading, var_heading_acc=process_var_heading_acc,
                          var_velocity=process_var_velocity, use_acc=use_acc)
        if position is not None:
            z = self.__measurement_update(position, imu_data, use_acc=use_acc)
            self.kf.R = set_R(var_position=variance_position, var_acc=variance_acceleration, var_heading=variance_heading,
                              use_acc=use_acc)
        else:
            z = self.kf.z
            self.kf.R = set_R(var_position=500, var_acc=500, var_heading=500,
                              use_acc=use_acc)  # High value, prefer process model

        u_rad = np.deg2rad(u[1])

        self.kf.predict(u=u_rad)
        self.kfupdate(z)

        # Values for estimated state as floats, showing two decimals
        x_kf = self.__float_with_2_decimals(self.kfx[0, 0])
        y_kf = self.__float_with_2_decimals(self.kfx[1, 0])

        if use_acc:
            yaw_kf = self.__float_with_2_decimals(self.kfx[2, 0])
            yaw_acc_kf = self.__float_with_2_decimals(self.kfx[3, 0])
            x_v_est = self.__float_with_2_decimals(self.kfx[4, 0])
            y_v_est = self.__float_with_2_decimals(self.kfx[5, 0])
            x_acc_est = self.__float_with_2_decimals(self.kfx[6, 0])
            y_acc_est = self.__float_with_2_decimals(self.kfx[7, 0])
        else:
            x_v_est = self.__float_with_2_decimals(self.kfx[2, 0])
            y_v_est = self.__float_with_2_decimals(self.kfx[3, 0])
            yaw_kf = self.__float_with_2_decimals(0)
            yaw_acc_kf = self.__float_with_2_decimals(0)
            x_acc_est = self.__float_with_2_decimals(0)
            y_acc_est = self.__float_with_2_decimals(0)

        log_likelihood = self.__float_with_2_decimals(self.kflog_likelihood)
        likelihood = self.__float_with_2_decimals(self.kflikelihood)

        filtered_loc = LocationData(x=x_kf, y=y_kf, z=0, quality=loc_quality)
        estimated_state = EstimatedState(filtered_loc,
                                         x_v_est=x_v_est,
                                         y_v_est=y_v_est,
                                         yaw_est=yaw_kf,
                                         yaw_acc_est=yaw_acc_kf,
                                         log_likelihood=log_likelihood,
                                         likelihood=likelihood,
                                         x_acc_est=x_acc_est,
                                         y_acc_est=y_acc_est)

        return estimated_state

    def __float_with_2_decimals(self, value):
        two_decimal_float = float("{0:.2f}".format(value))
        return two_decimal_float
