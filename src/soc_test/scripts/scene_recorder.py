#!/usr/bin/env python
import roslib; roslib.load_manifest('soc_test')

import os
import getopt
import sys
import json
import rospy
from soc_msg_and_serv.srv import *
from sensor_msgs.msg import *

BASELINE_PERCEPTION = 0.05

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

def normalize(lst):
        "Make sure the probabilities of all values sum to 1."
        total = sum(lst)
        if not (1.0-epsilon < total < 1.0+epsilon):
            for i in range(len(lst)):
                lst[i] /= total
        return lst

epsilon = 0.001

class SceneRecorder():
    """ Stores perceived scenes and their perception results as json file """

    def __init__(self, filename):

        rospy.init_node('scene_recorder', anonymous=True)

        self.filename = filename
        
        self.KINECT_OK = False
        self.camera_topic = "/chest_xtion/depth/points"
        #self.camera_topic = "/camera/depth/points"
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

                objects = list()
                types = dict()
                position = dict()
                orientation = dict()
                bbox = dict()
                
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

                    obj_key = key
                    print "-->", c.object_id, "is a", OBJT_MAP[OBJT_LIST[obj_key]]
                    confirm_text = "Confirm ('Enter'); Re-select (any key + 'Enter'). "
                    key = raw_input(confirm_text)
                    if key == "":
                        types[c.object_id] = OBJT_MAP[OBJT_LIST[obj_key]]
                        j = j + 1

                key = raw_input("Store scene ('Enter'); Discard scene (any key + 'Enter') ")
                if key != '':
                    print "Scene was discarded."
                    continue

                # generate scene description
                for i in range(len(response.classification)):

                    oid = response.classification[i].object_id
                    objects.append(oid)

                    pos = [response.centroid[i].x, response.centroid[i].y, response.centroid[i].z]
                    position[oid] = pos

                    # add default orientation as the current perception does not report any
                    orientation[oid] = [1.0, 0.0, 0.0, 0.0]

                    bbox_ = [] 
                    for p in response.bbox[i].point:
                        point = [p.x, p.y, p.z]
                        bbox_.append(point)

                    bbox[oid] = bbox_

                
                now = rospy.get_rostime()
                sid = str(now.secs)
                scene = {'scene_id' :    sid,
                         'objects' :     objects,
                         'type' :        types,
                         'position' :    position,
                         'orientation' : orientation,
                         'bbox':         bbox}

                # generate perception description

                percept = dict()

                                      
                for cls in response.classification:

                    objt = list()
                    conf = list()
                    for i in range(len(OBJT_LIST) - 1): # exclude class 'UNKNOWN'
                        t = OBJT_LIST[i]
                        c = BASELINE_PERCEPTION
                        if t in cls.type:
                            ind = cls.type.index(t)
                            c += cls.confidence[ind]

                        objt.append(OBJT_MAP[t])
                        conf.append(c)
                        
                    data = dict()
                    data['object_id'] = cls.object_id
                    data['type'] = objt
                    data['confidence'] = normalize(conf)


                    #print data
                    percept[cls.object_id] = data 
                
        
                # store scenes
                try:
                    with open(self.filename + '_scn.json', 'r+') as in_file:

                        try:
                            scns = json.load(in_file)
                        except ValueError:
                            print "VALUE ERROR - file does not contain valid json."
                            scns = list()
                except IOError:
                    print "IO ERROR - could not read from file. create new file."
                    scns = list()
                        
                    
                with open(self.filename + '_scn.json', 'w') as out_file:
                    scns.append(scene)
                    out_file.write(json.dumps(scns, out_file, indent=2))


                # store perception
                try:
                    with open(self.filename + '_perception.json', 'r+') as in_file:

                        try:
                            percepts = json.load(in_file)
                        except ValueError:
                            print "VALUE ERROR - file does not contain valid json."
                            percepts = dict()
                except IOError:
                    print "IO ERROR - could not read from file. create new file."
                    meta = dict()

                    objects = list()
                    for i in range(len(OBJT_LIST) - 1): # exclude class 'UNKNOWN'
                            objects.append(OBJT_MAP[OBJT_LIST[i]])

                    now = rospy.get_rostime()
                    meta['objects'] = objects
                    meta['perception_type'] = 'TUWs 3D object class recognition'
                    meta['random_seed'] = now.secs
                    meta['scene_file'] = self.filename + '_scn.json'
                    meta['stamp'] = now.secs

                    percepts = dict()
                    percepts['_meta'] = meta
                        
                    
                with open(self.filename + '_perception.json', 'w') as out_file:
                    percepts[sid] = percept
                    
                    out_file.write(json.dumps(percepts, out_file, indent=2))
                    
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

        sr = SceneRecorder(argv[1])
        sr.run()


    except Usage as err:
        print(err.msg)
        print("for help use --help")
    








