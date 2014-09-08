#!/usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from strands_perception_people_msgs.msg import PeopleTracker, PedestrianTrackingArray, Logging 
from geometry_msgs.msg import PoseStamped, Pose

class People_Tracker_Log():
    def __init__(self, people_tracker_topic, dataset_name):
        rospy.loginfo("Initialising Logging " + people_tracker_topic)
        self.pta = PedestrianTrackingArray()
        self.pt = PeopleTracker()
        self.rp = Pose()
        self.dataset_name = dataset_name 
        self.topic = people_tracker_topic
        self.msg_store = MessageStoreProxy(collection="people_perception_morse")
        # Subscribing to global poses of human avatars
        rospy.Subscriber(people_tracker_topic, PeopleTracker,
                self.pl_callback)
        # Subscribing to local poses from robot's point of view
        rospy.Subscriber('/morse_pedestrian_tracking', PedestrianTrackingArray,
                self.pt_callback)
        # Subscribing to robot pose
        rospy.Subscriber('/robot_pose', PoseStamped, self.rp_callback)
        # Publishing the log
        self.pub = rospy.Publisher('morse_people_tracker_log', Logging, queue_size=10)
        self.seq = 0


    def pl_callback(self,data):
        self.pt = data
    

    def pt_callback(self,data):
        self.pta = data
    

    def rp_callback(self,data):
        self.rp = data.pose


    def log(self):
        if len(self.pt.distances) == 0:
            return
        meta = dict() 
        meta['perspective'] = self.dataset_name 
        rospy.logdebug("Person detected for " + self.topic)
        message = Logging()
        message.header.seq = self.seq
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = '/map'
        message.uuids = self.pt.uuids
        message.people = self.pt.poses
        message.people_tracker = self.pt
        #message.pedestrian_tracking = self.pta.pedestrians
        message.robot = self.rp
        self.msg_store.insert(message, meta)
        self.pub.publish(message)
        self.seq += 1 
    

if __name__ == '__main__':
    rospy.init_node('morse_people_tracker_log', anonymous=True)
    ptlm = People_Tracker_Log('/morse_people_tracker', 'global')
    ptlm_robot = People_Tracker_Log('/morse_people_tracker_robot_view', 'robot')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ptlm.log()
        ptlm_robot.log()
        rate.sleep()
    

