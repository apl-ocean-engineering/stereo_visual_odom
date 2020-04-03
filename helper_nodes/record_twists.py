import argparse
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class PoseRecorder:
    def __init__(self, save_folder):
        self.save_folder = save_folder
        self.name1 = "lsd_slam_raw"
        self.name2 = "filtered_odom"
        self.name3 = "stereo_odom"

        self.odomList = [0]

        filtered = Subscriber(
            "/odometry/filtered", Odometry)
        lsd = Subscriber(
            "/lsd_slam/twist", TwistWithCovarianceStamped)
        so = Subscriber(
            "/stereo_odometry/twist", TwistWithCovarianceStamped)

        ats = ApproximateTimeSynchronizer([filtered, lsd, so],
                                          queue_size=5, slop=0.1)

        ats.registerCallback(self.twist_callback)

        rospy.spin()

    def odom2twsitStamped(self, odom):
        odomTwist = TwistWithCovarianceStamped()
        odomTwist.twist = odom.twist
        odomTwist.twist = odom.twist
        odomTwist.header = odom.header

        return odomTwist

    def twist_callback(self, msg1, msg2, msg3):
        print('here')
        if self.odomList is not None:
            if 0 in self.odomList:
                msg1 = self.odom2twsitStamped(msg1)
            if 1 in self.odomList:
                msg2 = self.odom2twsitStamped(msg2)
            if 2 in self.odomList:
                msg2 = self.odom2twsitStamped(msg3)

        self.save(msg1, self.name1)
        self.save(msg2, self.name2)
        self.save(msg3, self.name3)

    def save(self, msg, name):
        current_secs = msg.header.stamp.secs
        current_nsecs = msg.header.stamp.nsecs

        secs = current_secs + current_nsecs/1e9

        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        z = msg.twist.twist.linear.z
        roll = msg.twist.twist.angular.x
        pitch = msg.twist.twist.angular.y
        yaw = msg.twist.twist.angular.z

        save_line = str(secs) + "," + str(x) + "," + str(y) + "," + \
            str(z) + "," + str(roll) + "," + str(pitch) + "," + \
            str(yaw)

        self.current_frame_save_file = open(
                 self.save_folder + name + '.txt', 'a+')

        self.current_frame_save_file.write(save_line + '\n')


if __name__ == '__main__':
    rospy.init_node("pose_recorder")
    parser = argparse.ArgumentParser("Record LSD-SLAM pose infmormation")
    parser.add_argument("save_folder")

    args = parser.parse_args()
    PR = PoseRecorder(args.save_folder)
