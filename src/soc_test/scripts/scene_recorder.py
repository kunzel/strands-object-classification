#!/usr/bin/env python
import roslib; roslib.load_manifest('soc_test')

import os
import getopt
import sys
import json
import rospy
from soc_msg_and_serv.srv import *
from sensor_msgs.msg import *
import tf


# Import opencv
import cv
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError  # to convert sensor_msgs to OpenCV image
from roslib import message
from geometry_msgs.msg import Point32
from image_geometry import PinholeCameraModel
BASELINE_PERCEPTION = 0.05

OBJT_LIST = ['book',
             'bottle',
             #'cellphone',
             'keyboard',
             'monitor',
             'mouse',
             'mug',
             'notebook',
             'telephone',
             'UNKNOWN']

OBJT_MAP = {'book' : 'Book',        
            'bottle' : 'Bottle',
            #'cellphone' : 'Mobile',
            'keyboard' : 'Keyboard',
            'monitor' : 'Monitor',
            'mouse' : 'Mouse',
            'mug' :  'Mug',
            'notebook' : 'Laptop',
            'telephone' : 'Telephone',
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
        self.bridge=CvBridge()

        self._tf_listener = tf.TransformListener(60)
        self.filename = filename

        self.KINECT_OK = False
        self.camera_topic = "/chest_xtion/depth/points"
        self.camera_image_topic = "/chest_xtion/rgb/image_color"
        self.camera_image_info_topic = "/chest_xtion/rgb/camera_info"
               #self.camera_topic = "/camera/depth/points"
        self.kinect_trials = 5
        self.check_kinect()
        cv.NamedWindow("Image window")
        self.GET_POINTCLOUD = False
        

    def check_pointcloud(self, pc2):
        self.KINECT_OK = True

    def check_kinect(self):

        s=rospy.Subscriber(self.camera_topic, PointCloud2, self.check_pointcloud)

        trials = 0

        r = rospy.Rate(1.0)

        while not self.KINECT_OK:
            rospy.loginfo("Checking kinect status...")

            if (trials >= self.kinect_trials):
                rospy.logerror("Kinect is not working")
                return
            r.sleep()

        rospy.loginfo("Kinect is up and running")
        s.unregister()

    def get_pointcloud(self, pc2):
        isinstance(pc2, PointCloud2)
        if self.GET_POINTCLOUD:
            self.pointcloud = pc2
            self.GET_POINTCLOUD = False
            self.GOT_POINTCLOUD = True

    def image_cb(self,image):
        try:
            self._image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e
            
    def image_info_cb(self, image_info):
        self._image_info =  PinholeCameraModel()
        self._image_info.fromCameraInfo(image_info)
        isinstance(self._image_info, PinholeCameraModel)


    def run(self):

        frame = 0
        key = ""
        finished = False
        while not rospy.is_shutdown() and not finished:
            frame += 1

            if key != '':
                break

            self.GOT_POINTCLOUD = False
            self.GET_POINTCLOUD = True
            
            r = rospy.Rate(1.0)
            
            # Get an image
            self._image = None
            sub = rospy.Subscriber(self.camera_image_topic, Image, self.image_cb)
            while self._image is None:
                rospy.loginfo("Waiting for image...")
                r.sleep()
            sub.unregister()
            rospy.loginfo("Got one.")
            cv.ShowImage("Image window", cv.fromarray(self._image))
            cv.WaitKey(100)
            
            # Get image info
            self._image_info = None
            sub = rospy.Subscriber(self.camera_image_info_topic, CameraInfo, self.image_info_cb)
            while self._image_info is None:
                rospy.loginfo("Waiting for image calibration...")
                r.sleep()
            rospy.loginfo("Got it.")
            sub.unregister()

            text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 1, 1)
            cv.PutText(cv.fromarray(self._image), "Scene: %d"%frame,(20, 20), 
                       text_font, (255, 0, 0, 255))
        
            # Got pointcloud
            sub=rospy.Subscriber(self.camera_topic, PointCloud2, self.get_pointcloud)
            while not self.GOT_POINTCLOUD:
                rospy.loginfo("Wait for pointcloud...")
                r.sleep()
            sub.unregister()
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
                
                rospy.loginfo("Got clusters.")
                
                # Project each response cluster into image
                box_locations = []
                for cloud in response.cloud:
                    assert isinstance(cloud, PointCloud2)
                    print cloud.fields
                    pc1 = PointCloud()
                    pc1.header = cloud.header
                    # hack the time! dont move the robot :-0
                    pc1.header.stamp = rospy.Time.now()
                    print type(cloud)
                    pc1.points = [Point32(*p) for p in pc2.read_points(cloud)]
                    #print pc1.points
                    self._tf_listener.waitForTransform(pc1.header.frame_id,
                                                       self._image_info.tf_frame, 
                                                       pc1.header.stamp,
                                                       rospy.Duration(4))
                    image_frame_cloud = self._tf_listener.transformPointCloud(
                        self._image_info.tf_frame, 
                        pc1)
                    min_x, max_x, min_y, max_y =  640, 0, 480, 0
                    for pt in image_frame_cloud.points:
                        u, v = self._image_info.project3dToPixel((pt.x, pt.y, pt.z))
                        if v < min_y:
                            min_y = int(v)
                        if v > max_y:
                            max_y = int(v)
                        if u < min_x:
                            min_x = int(u)
                        if u > max_x:
                            max_x = int(u)
                    box_locations.append(((min_x, min_y), (max_x, max_y)))
                    cv.Rectangle(cv.fromarray(self._image), (min_x, min_y), (max_x, max_y), (255, 0, 0, ), 2)
                    
                    rospy.loginfo("Transformed cloud into camera")
                cv.ShowImage("Image window", cv.fromarray(self._image))
                cv.WaitKey(100)   
                
                objects = list()
                types = dict()
                position = dict()
                orientation = dict()
                bbox = dict()
                
                labelling = True
                labels = [None] * len(response.classification)
                import copy
                display_image = self._image.copy()
                def update_display(accept_text=False):
                    global display_image
                    display_image = self._image.copy()
                    text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 0.7, 0.7)
                    for location, label in zip(box_locations, labels):
                        if label is not None:
                            cv.PutText(cv.fromarray(display_image), OBJT_LIST[label], (location[0][0], location[0][1]-10),
                                       text_font, (255, 0, 255, 255))
                            cv.Rectangle(cv.fromarray(display_image),
                                         location[0], location[1], (255, 255, 0, ), 4)
                    if accept_text:
                        text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 0.7, 0.7)
                        cv.PutText(cv.fromarray(display_image), "Accept => SPACE; Discard => ESC, Finish=>q", (0, 30),
                                   text_font, (255, 255, 255, 255))

                    return display_image
                def mouse_click(event, x, y, flags, param):
                    global display_image
                    #i = 0
                    for i, (b_l, b_u) in sorted(enumerate(box_locations),
                                                key=lambda x:(x[1][1][1]-x[1][0][1])*(x[1][1][0]-x[1][0][0])):
                        if (event==cv.CV_EVENT_LBUTTONDOWN and
                            x < b_u[0] and  x > b_l[0] and
                            y <  b_u[1] and y > b_l[1]):
                            if labels[i] is None:
                                labels[i] = -1
                            labels[i] += 1
                            if labels[i] > len(OBJT_LIST) - 1:
                                labels[i] = 0
                            update_display()
                            break
                        if (event==cv.CV_EVENT_RBUTTONDOWN and
                            x < b_u[0] and  x > b_l[0] and
                            y <  b_u[1] and y > b_l[1]):
                            if labels[i] is None:
                                labels[i] = 0
                            labels[i] -= 1
                            if labels[i] < 0:
                                labels[i] = len(OBJT_LIST) - 1
                            update_display()
                            break
                        #i += 1
                cv.SetMouseCallback("Image window", mouse_click, None)
                while labelling:
                    cv.ShowImage("Image window", cv.fromarray(update_display(True)))
                    k = cv.WaitKey(100)
                    print k
                    if k == 1048608 : # SPACE:=> Done
                        if any([True if i is None else False for i in labels]):
                            rospy.logwarn("Can't proceed,  unlabled clusters!")
                        else:
                            break
                    if k == 1048603:  # ESC:=> skip
                        labelling = False
                    if k == 1048689:  # q:=> QUit
                        labelling = False
                        finished = True
                else:
                    continue
                        
                        #break
                #sys.exit(1)
                
                for c, tp in zip(response.classification, labels):
                    types[c.object_id] = OBJT_MAP[OBJT_LIST[tp]]
                
                #j = 0
                #while j < len(response.classification):

                    #c = response.classification[j]

                    #print '--- [ SELECT A TYPE ] --- '
                    #for i in range(0,len(OBJT_LIST)):
                        #print i, OBJT_LIST[i]

                    #cluster_text = 'Enter object type for ' + c.object_id + ': '
                    #key = ''
                    #while key not in range(0,len(OBJT_LIST)):
                        #try:
                            #key = int(raw_input(cluster_text))
                        #except ValueError:
                            #continue
                        #continue

                    #obj_key = key
                    #print "-->", c.object_id, "is a", OBJT_MAP[OBJT_LIST[obj_key]]
                    #confirm_text = "Confirm ('Enter'); Re-select (any key + 'Enter'). "
                    #key = raw_input(confirm_text)
                    #if key == "":
                        #types[c.object_id] = OBJT_MAP[OBJT_LIST[obj_key]]
                        #j = j + 1

                #key = raw_input("Store scene ('Enter'); Discard scene (any key + 'Enter') ")
                #if key != '':
                    #print "Scene was discarded."
                    #continue
                
                display_image = update_display()
                txt = "Store scene => 'SPACE'; Discard scene => 'ESC'"
                text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 0.7, 0.7)
                cv.PutText(cv.fromarray(display_image), txt, (0, 60),
                           text_font, (255, 255, 255, 255))
                waiting = True
                while waiting:
                    cv.ShowImage("Image window", cv.fromarray(display_image))
                    k = cv.WaitKey(100)
                    if k == 1048608 : # SPACE:=> Done
                        break
                    if k == 1048603:  # ESC:=> skip
                        waiting = False
                else:
                    print "Scene discarded"
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
                
            display_image = update_display()
            txt = "Record another scene => 'SPACE'; Finish => 'ESC'"
            text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 0.7, 0.7)
            cv.PutText(cv.fromarray(display_image), txt, (0, 60),
                       text_font, (255, 255, 255, 255))
            waiting = True
            while waiting:
                cv.ShowImage("Image window", cv.fromarray(display_image))
                k = cv.WaitKey(100)
                if k == 1048608 : # SPACE:=> Done
                    key = ""
                    break
                if k == 1048603:  # ESC:=> skip
                    waiting = False
            else:
                break
                
#            key = raw_input("Record new scene ('Enter'); Quit (any key + 'Enter') ")


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







