#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros


class GazeboGroundTruth:
    """
    This code publishes a transform from the model to map/world frame.
    Use this tf to validate other sensor errors.
        link_name: finds
    """
    def __init__(self, model_name, model_new_name, publish_pose=True):
        self.model_name = model_name
        self.model_new_name = model_new_name
        self.model_pose = PoseStamped()
        self.br = tf2_ros.TransformBroadcaster()

        if not self.model_name:
            raise ValueError("'model_name' is an empty string")

        self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.pose_pub = rospy.Publisher("/gazebo/" + self.model_new_name + "/pose", PoseStamped, queue_size=10)

    def callback(self, data):
        try:
            ind = data.name.index(self.model_name)
            self.model_pose.pose = data.pose[ind]
            self.model_pose.header.frame_id = "map"
            self.model_pose.header.stamp = rospy.Time.now()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map" # from
            t.child_frame_id = self.model_new_name # to
            t.transform.translation = self.model_pose.pose.position
            t.transform.rotation = self.model_pose.pose.orientation
            self.br.sendTransform(t)

        except ValueError:
            pass


if __name__ == '__main__':
    rospy.init_node('gazebo_ground_truth', anonymous=True)

    # iris and aruco marker models transform are created
    gp = GazeboGroundTruth('iris', "ground_truth_iris", publish_pose=True)
    gp_aruco = GazeboGroundTruth('aruco_visual_marker_0', 'ground_truth_aruco')

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        gp.pose_pub.publish(gp.model_pose)
        gp_aruco.pose_pub.publish(gp_aruco.model_pose)
        rate.sleep()
