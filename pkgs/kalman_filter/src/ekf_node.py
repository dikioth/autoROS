import rospy
import numpy as np
# from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from Kalman import Kalman


class Estimator:
    def __init__(self):
        # subsribing to DWM1001 and IMU data.
        rospy.init_node('autoros_ekf', anonymous=True)

        # Subscribe to Odometry and imu topics.
        rospy.Subscriber("/dwm1001/tag_odom", Odometry, self.tag_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        # Creating publisher to broadcast the position estimation.
        self.odom_publisher = rospy.Publisher(
            "/kalman_filter/odom_estimation", Odometry)

        self.rate = rospy.Rate(10)  # Publish rate 10Hz
        # Atributes
        self.odom = Odometry
        self.imu = Imu

        # Creating Kalman object
        self.kalman = Kalman(self.odom, self.imu, 1e-3)

    def main(self):

        rospy.loginfo("Running ...")

    def tag_callback(self, data):
        self.odom = data
        # self.loc_data = LocationData(x, y, 1.5, 100)

    def imu_callback(self, data):
        self.imu = data

    # def set_steering(self, u_yaw):
    #     self.tan_u = np.tan(u_yaw)

    def loop(self):
        self.kalman.kalman_predic()
        self.kalman.kalman_update(self.odom, self.imu)
        rospy.loginfo(self.kalman.get_estimation())
        self.rate.sleep()


if __name__ == "__main__":
    rospy.loginfo("Starting ...")
    e = Estimator()
    e.main()

    try:
        while not rospy.is_shutdown():
            e.loop()

    except Exception as e:
        print(e)
