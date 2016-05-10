#!/usr/bin/env python
import cv2, rospy, roslib, sys
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
roslib.load_manifest('team_3_robot_follower')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from team_3_robot_follower.msg import all_faces
import numpy as np
from pykalman import KalmanFilter
import re
from team_3_robot_follower.msg import tracking

HEIGHT = 480
WIDTH = 640

current_measurement = [0, 0]
face_center_pixels = [0, 0]
face = [0, 0]
image = np.zeros((HEIGHT,WIDTH,3), np.uint8)
init_face = 0
face_id = ''
tracked_person = 'Mainak'

x_g = []
y_g = []
z_g = []
child_id = []

def tf_cb(data):
    global current_measurement
    global face
    global init_face
    global face_id
    global x_g
    global y_g
    global z_g
    global child_id
    #global face_center_pixels
    #x = []
    #y = []
    #z = []
    head_pixels = []
    dist = []
    #if(data.transforms[0].child_frame_id == 'head_1'):
    if init_face < 2 and (face_center_pixels != [0, 0]):
        if(re.match('head_.', data.transforms[0].child_frame_id)):
            if (re.match('head_1', data.transforms[0].child_frame_id)):
                if init_face == 0:
                    x_g = []
                    y_g = []
                    z_g = []
                    child_id = []
                elif init_face == 1:
                    for i in xrange(len(x_g)):
                        head_pixels.append(np.asarray(calibrate(y_g[i], z_g[i])))
                    print 'head_pixels: ', head_pixels
                    print 'fc_pixels: ', face_center_pixels

                    face_center_pexels_array = np.asarray(face_center_pixels)
                    
                    for i in xrange(len(x_g)):
                        dist.append(np.linalg.norm(head_pixels[i]-face_center_pexels_array))
                    print 'Dist: ', dist
                    idx = np.argmin(np.asarray(dist))
                    face = calibrate(y_g[idx], z_g[idx])
                    current_measurement = [x_g[idx], y_g[idx]]
                    face_id = child_id[idx]
                    print 'face id: ', face_id
                    print 'idx: ', idx
                    print 'init_face: ', init_face
                init_face += 1 
            #position = data.tansforms[0].transform.translation
            print 'x: %f, y: %f, z: %f'% (data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z)
            #current_measurement = [data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y]
            print 'Child Frame ID: ', data.transforms[0].child_frame_id
            x_g.append(data.transforms[0].transform.translation.x)
            y_g.append(data.transforms[0].transform.translation.y)
            z_g.append(data.transforms[0].transform.translation.z)
            child_id.append(data.transforms[0].child_frame_id)
    elif init_face == 2:
        #print 'init_face: ', init_face
        if(re.match(face_id, data.transforms[0].child_frame_id)):
            x = data.transforms[0].transform.translation.x
            y = data.transforms[0].transform.translation.y
            z = data.transforms[0].transform.translation.z
            face = calibrate(y, z)
            current_measurement = [x, y]

    #if (len(x_g) > 0):
    #    print '\n'
    #if (len(x) > 0):
    #    if (len(x) > 1):
    #        print 'Length: ', len(x)
    #        #current_measurement = [x[0], y[0]]
    #    for i in xrange(len(x)):
    #        head_pixels.append(np.asarray(calibrate(y[i], z[i])))

    #    face_center_pexels_array = np.asarray(face_center_pixels)
    #
    #    for i in xrange(len(x)):
    #        dist.append(np.linalg.norm(head_pixels[i]-face_center_pexels_array))
    #    idx = np.argmin(np.asarray(dist))
    #    #print 'Idx: ', idx
    #    face = calibrate(y[idx], z[idx])
    #    current_measurement = [x[idx], y[idx]]
        

def image_cb(data):
    global image
    try:
      image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
def face_cb(data):
    global face_center_pixels
    #print 'face boundary: ', data
    for i in xrange(len(data.x)):
        if (data.s[i] == tracked_person):
            center_x = data.x[i] + np.ceil(data.w[i]/2)
            center_y = data.y[i] + np.ceil(data.h[i]/2)
            face_center_pixels = [int(center_x), int(center_y)]

def calibrate(x,y):
    x_im = (WIDTH /2.1)  * (-x) + (WIDTH/2)
    y_im = (HEIGHT/1.03) * (-y) + (HEIGHT*0.57/1.03)
    return [int(x_im), int(y_im)] 

def trackKalman():

    l_publ = rospy.Publisher("tracking", tracking, queue_size = 10)

    rate = rospy.Rate(10) # 10hz

    initstate = [current_measurement[0], current_measurement[1], 0, 0]
    Transition_Matrix=[[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]]
    Observation_Matrix=[[1,0,0,0],[0,1,0,0]]
    initcovariance=1.0e-3*np.eye(4)
    transistionCov=1.0e-4*np.eye(4)
    observationCov=1.0e-1*np.eye(2)
    kf=KalmanFilter(transition_matrices=Transition_Matrix,
                observation_matrices =Observation_Matrix,
                initial_state_mean=initstate,
                initial_state_covariance=initcovariance,
                transition_covariance=transistionCov,
                observation_covariance=observationCov)

    start = 1
    t = 1
    while not rospy.is_shutdown():

        #cv2.rectangle(image,(face[0]-50,face[1]-50),(face[0]+50,face[1]+50),(255,0,0),2)
        ##cv2.rectangle(image,(face_center_pixels[0]-50,face_center_pixels[1]-50),(face_center_pixels[0]+50,face_center_pixels[1]+50),(255,0,0),2)
        #cv2.imshow("Calibrated Boundary", image)
        #cv2.waitKey(1)

        if (start == 1):
            start = 0
            filtered_state_means = initstate
            filtered_state_covariances = initcovariance
        
        print 'current measurement: ', current_measurement
        (pred_filtered_state_means, pred_filtered_state_covariances) = kf.filter_update(filtered_state_means, filtered_state_covariances, current_measurement);
        t += 1
        filtered_state_means = pred_filtered_state_means
        filtered_state_covariances = pred_filtered_state_covariances
        print 'predicted: ', filtered_state_means[0], filtered_state_means[1]
        #print type(current_measurement[0])
        #print type(filtered_state_means[0])
        location = tracking()
        location.x0 = current_measurement[0]
        location.y0 = current_measurement[1]
        location.x1 = filtered_state_means[0]
        location.y1 = filtered_state_means[1]
        l_publ.publish(location)
        print '\n'
        rate.sleep()

def get_tf():

    rospy.init_node('get_tf', anonymous=True)

    rospy.Subscriber('/faces', all_faces, face_cb)
    rospy.Subscriber("/camera/rgb/image_color", Image, image_cb)
    rospy.Subscriber("/tf", TFMessage, tf_cb)

    trackKalman()

    rospy.spin()

if __name__ == '__main__':
    get_tf()
