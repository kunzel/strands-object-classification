#!/usr/bin/env python
import roslib; roslib.load_manifest('soc_test')

import getopt
import sys
import json
import rospy
from soc_msg_and_serv.srv import *
from sensor_msgs.msg import *

OBJT_LIST = ['book',
             'bottle',
             'cellphone',
             'keyboard',
             'monitor',
             'mouse',
             'mug',
             'notebook',
             'UNKNOWN']

OBJT_MAP = {'book' : 'Book',        
            'bottle' : 'Bottle',
            'cellphone' : 'Mobile',
            'keyboard' : 'Keyboard',
            'monitor' : 'Monitor',
            'mouse' : 'Mouse',
            'mug' :  'Mug',
            'notebook' : 'Laptop',
            'UNKNOWN' : 'UNKNOWN'
            }


class SceneRecorder():
    """ Stores perceived scenes and their perception results as json file """

    def __init__(self, filename):

        rospy.init_node('scene_recorder', anonymous=True)

        self.filename = filename
        
        self.KINECT_OK = False
        self.camera_topic = "/camera/depth/points"
        self.kinect_trials = 5
        self.check_kinect()

        self.GET_POINTCLOUD = False

    def check_pointcloud(self, pc2):
        self.KINECT_OK = True

    def check_kinect(self):

        rospy.Subscriber(self.camera_topic, PointCloud2, self.check_pointcloud)
        
        trials = 0

        r = rospy.Rate(1.0)
        
        while not self.KINECT_OK:
            rospy.loginfo("Checking kinect status...")

            if (trials >= self.kinect_trials):
                rospy.logerror("Kinect is not working")
                return
            r.sleep()
            
        rospy.loginfo("Kinect is up and running")


    def get_pointcloud(self, pc2):

        if self.GET_POINTCLOUD:
            self.pointcloud = pc2
            self.GET_POINTCLOUD = False
            self.GOT_POINTCLOUD = True

            
    def run(self):

        rospy.Subscriber(self.camera_topic, PointCloud2, self.get_pointcloud)
        
        while not rospy.is_shutdown():

            key = raw_input("Record new scene ('Enter'); Quit (any key + 'Enter') ")

            if key != '':
                break

            self.GOT_POINTCLOUD = False
            self.GET_POINTCLOUD = True

            r = rospy.Rate(1.0)
            
            while not self.GOT_POINTCLOUD:
                rospy.loginfo("Wait for pointcloud...")
                r.sleep()


            rospy.loginfo("Got pointcloud!")
            # call classification service with pointcloud
            service_name = 'segment_and_classify'
            rospy.wait_for_service(service_name)
        
            try:
                rospy.loginfo("Call service ...")
                
                sac = rospy.ServiceProxy(service_name, segment_and_classify)
            
                request = segment_and_classifyRequest()
                request.cloud = self.pointcloud
            
                response = sac(request)

                j = 0
                while j < len(response.classification):

                    c = response.classification[j]

                    print '--- [ SELECT A TYPE ] --- '
                    for i in range(0,len(OBJT_LIST)):
                        print i, OBJT_LIST[i]

                    cluster_text = 'Enter object type for ' + c.object_id + ': '
                    key = ''
                    while key not in range(0,len(OBJT_LIST)):
                        try:
                            key = int(raw_input(cluster_text))
                        except ValueError:
                            continue
                        continue

                    print "-->", c.object_id, "is a", OBJT_MAP[OBJT_LIST[key]]
                    confirm_text = "Confirm ('Enter'); Re-select (any key + 'Enter'). "
                    key = raw_input(confirm_text)
                    if key == "":
                        j = j + 1

                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e



class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def help_msg():
    return """
  Usage: scene_recorder.py [-h] <file_prefix>

    file_prefix        prefix used for all generated files containing: scenes, classification results and point clouds

    -h, --help for seeing this msg
"""

if __name__ == "__main__":
    argv = None
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "h", ["help"])
        except getopt.error as msg:
            raise Usage(msg)

        if ('-h','') in opts or ('--help', '') in opts or len(args) is not 1:
            raise Usage(help_msg())

        sr = SceneRecorder(argv[0])
        sr.run()


    except Usage as err:
        print(err.msg)
        print("for help use --help")
    








