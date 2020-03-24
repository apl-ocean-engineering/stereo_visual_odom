import argparse
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import TwistWithCovarianceStamped


class PoseRecorder:
    def __init__(self, save_folder):
        self.save_folder = save_folder
        self.name1 = "lsd_slam"
        self.name2 = "stereo_odom"

        so = Subscriber(
            "/stereo_odometry/twist", TwistWithCovarianceStamped)
        lsd = Subscriber(
            "/lsd_slam/twist", TwistWithCovarianceStamped)

        ats = ApproximateTimeSynchronizer([so, lsd],
                queue_size=5, slop=0.1)

        ats.registerCallback(self.twist_callback)

        rospy.spin()

    def twist_callback(self, msg1, msg2):
        self.save(msg1, self.name1)
        self.save(msg2, self.name2)

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
