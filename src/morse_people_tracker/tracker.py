#!/usr/bin/env python
"""
    A simple people tracker program getting ground truth from MORSE Pose.
    The trackers are published via ROS message following the format
    PeopleTracker.msg
    Global and robot's perspective are provided
"""
import math
import re
import rospy
import rosgraph.masterapi
from strands_perception_people_msgs.msg import PeopleTracker 
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

detected_human_poses = []
human_poses = []

class Human_Pose:
    counter = 0

    def __init__(self, name, x_offset = 0.0, y_offset = 0.0):
        self.name = name 
        self.index = Human_Pose.counter 
        self.uuids = str(rospy.Time.now()) + '-' + name + '-' + str(self.index)
        self.pose = Pose()
        Human_Pose.counter += 1
        self.x_off = x_offset
        self.y_off = y_offset

    def callback(self, data):
        data.pose.position.x += self.x_off
        data.pose.position.y += self.y_off
        self.pose = data.pose 


def semcam_callback(data):
    global human_poses, detected_human_poses
    data = data.data[1:len(data.data)-1]
    index_name = data.find("name")

    detected_human_poses = []

    while index_name > -1:
        # getting name, every object must have a name
        name = ''
        if index_name != -1:
            name = data[index_name+8:]        
            name = name[0:name.find("\"")]

        # update detected human from robot's perspective 
        if name != '':
            for i in human_poses:
                if i.name == name:
                    #temp = i
                    #temp.pose = pose
                    #detected_human_poses.append(temp)
                    detected_human_poses.append(i)
                    break
        
        data = data[index_name+10:]
        index_name = data.find("name")


def people_tracker(x_offset = 0.0, y_offset = 0.0):
    global human_poses, detected_human_poses
    # add robot pose and listen to robot/pose in strands_morse bham_cs_morse (cs_lg.py)
    robot_pose = Human_Pose('robot_pose', x_offset, y_offset)
    rospy.Subscriber('/robot_pose/', PoseStamped, robot_pose.callback)
    # subscribe to semcam to get pose recorded by semantic camera
    rospy.Subscriber('/semcam', String, semcam_callback)
    
    # subscribe to human pose from strands_morse bham_cs_morse (cs_lg.py) where the name format for
    # each human is human[number]/pose[number]
    master = rosgraph.masterapi.Master('/rostopic')
    for i in master.getPublishedTopics('/'):
        matchObj = re.search(r'human([0-9]+)/pose\1', i[0], re.M)
        if matchObj:
            name = i[0]
            human_pose = Human_Pose(name[1:name[1:].find("/")+1],x_offset,y_offset) 
            human_poses.append(human_pose)
            rospy.Subscriber(i[0], PoseStamped, human_pose.callback)

    # publish pedestrian locations from global perspective and robot's
    # perspective
    pub = rospy.Publisher('morse_people_tracker', PeopleTracker, queue_size=10)
    pub_robot = rospy.Publisher('morse_people_tracker_robot_view',
            PeopleTracker, queue_size=10)

    # constructing the message every 100 miliseconds
    r = rospy.Rate(10) # 10Hz
    seq = 0
    while not rospy.is_shutdown():  
        # message for global perspective
        message = PeopleTracker()
        message.header.seq = seq
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = '/map'
        message.uuids = [x.uuids for x in human_poses]
        message.poses = [x.pose for x in human_poses]
        message.distances = [math.hypot(i.pose.position.x - robot_pose.pose.position.x,
            i.pose.position.y - robot_pose.pose.position.y) for i in human_poses]
        message.min_distance = min(message.distances)
        message.angles = [math.atan2(i.pose.position.y -
            robot_pose.pose.position.y, i.pose.position.x -
            robot_pose.pose.position.x) for i in human_poses]
        message.min_distance_angle = message.angles[message.distances.index(min(message.distances))]
        pub.publish(message)

        # message for robot's perspective
        message = PeopleTracker()
        message.header.seq = seq
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = '/map'
        message.uuids = [x.uuids for x in detected_human_poses]
        message.poses = [x.pose for x in detected_human_poses]
        message.distances = [math.hypot(i.pose.position.x - robot_pose.pose.position.x,
            i.pose.position.y - robot_pose.pose.position.y) for i in detected_human_poses]
        message.angles = [math.atan2(i.pose.position.y -
            robot_pose.pose.position.y, i.pose.position.x -
            robot_pose.pose.position.x) for i in detected_human_poses]
        if len(message.distances) > 0:
            message.min_distance = min(message.distances)
            message.min_distance_angle = message.angles[message.distances.index(min(message.distances))]
        pub_robot.publish(message)

        seq += 1
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('morse_people_tracker', anonymous=True)
    #x = rospy.get_param("~x_offset", 19.482)
    #y = rospy.get_param("~y_offset", -8.596)
    try:
        people_tracker()
    except rospy.ROSInterruptException: pass
