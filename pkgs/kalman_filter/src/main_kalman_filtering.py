import rospy
import numpy as np
from custom_msgs import *
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from PositionEstimator import PositionEstimator


class Estimator:
    def __init__(self):
        # subsribing to DWM1001 and IMU data.
        context = Context("./default_settings.json")
        self.position_estimator = PositionEstimator(
            **context.settings["kalman"])
        self.yaw = 0
        self.tan_u = 0
        self.start = True

    def main(self):
        rospy.init_node('autoros_ekf', anonymous=True)
        rospy.Subscriber("/dwm1001/tag_odom", Odometry, self.tag_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.loginfo("Running ...")
        rospy.spin()

    def tag_callback(self, data):
        self.meas.x = data.pose.pose.position.x
        self.meas.y = data.pose.pose.position.y

        self.loc_data = LocationData(x, y, 1.5, 100)

    def imu_callback(self, data):
        # imu callback

        real_acc = Transform(0, 0, 0)
        world_acc = Transform(data.linear_acceleration.x,
                              a_w_y.linear_acceleration.y, 0)

        rpy_radians = euler_from_quaternion(
            data.orientation)  # RPY in radians
        rpy_degree = [x * 360 / (2*np.pi) for x in rpy_radians]  # RPY degrees
        yaw = rpy_degree[2]
        if yaw < 0:
            yaw += 360
        rotation = Rotation(yaw, 0, 0)

        self.imu_data = IMUData(
            rotation=rotation, real_acceleration=real_acc, world_acceleration=world_acc)

    def set_steering(self, u_yaw):
        self.tan_u = np.tan(u_yaw)

    def loop(self):

        measurement = Measurement(self.loc_data, self.imu_data)
        control_signal = ControlSignal(0, self.tan_u)
        if self.start:
            position_estimator.start_kalman_filter(loc_data)
            self.start = False

        estimated_state = position_estimator.do_kalman_updates(
            self.loc_data, self.imu_data, control_signal=control_signal.to_numpy())
        estimated_state.measurement = measurement
        rospy.loginfo("EST LOC: " + estimated_state.location_est)


if __name__ == "__main__":
    rospy.loginfo("Starting ...")
    e = Estimator()
    e.main()
    try:
        while not rospy.is_shutdown():
            e.loop()

    except Exception as e:
        print(e)
