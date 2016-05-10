#!/usr/bin/env python
import roslib
roslib.load_manifest('team_3_robot_follower')
import sys, os, os.path
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cPickle as pkl
from team_3_robot_follower.msg import all_faces

# change this as required
PATH = os.path.dirname(os.path.abspath(__file__)) + '/'

cascadePath = PATH+'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(cascadePath)
recognizer = cv2.createFisherFaceRecognizer()
recognizer_lb = cv2.createLBPHFaceRecognizer()
TRAIN_SIZE = 300
TEST_SIZE = 200

class face_node:

  def __init__(self):
    self.f_publ = rospy.Publisher("faces", all_faces, queue_size=10)
    self.is_image_present = 0

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.image = cv_image
    # self.gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    self.is_image_present = 1

  def detect_face(self):

    height = 480
    width = 640
    rate = rospy.Rate(10)
    cv2.namedWindow("Face Detector", 1)

    while not rospy.is_shutdown():
      if(self.is_image_present):
        im = self.image
      else:
        im = np.ones((height,width,3), np.uint8)
      (rows,cols,channels) = im.shape
      im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
      train_faces_news = face_cascade.detectMultiScale(im_gray, 1.3, 5)
      for (x,y,w,h) in faces:
        cv2.rectangle(im,(x,y),(x+w,y+h),(255,0,0),2)
        face_gray = np.array(im_gray[y:y+h, x:x+w], 'uint8')
      
      cv2.imshow("Face Detector", im)
      cv2.waitKey(3)
      rate.sleep()

  def prepare_face_data(self, face_label):

    height = 480
    width = 640
    rate = rospy.Rate(10)
    facecount = 0
    face_images = []
    face_labels = []
    cv2.namedWindow("Training Set Preparation", 1)

    while (not rospy.is_shutdown()) and (facecount < TRAIN_SIZE+TEST_SIZE):
      if(self.is_image_present):
        im = self.image
      else:
        im = np.ones((height,width,3), np.uint8)
      (rows,cols,channels) = im.shape
      im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
      faces = face_cascade.detectMultiScale(im_gray, 1.3, 5)
      if (len(faces) == 1):
        for (x,y,w,h) in faces:
          if ((w>50) and (h>50)):
            cv2.rectangle(im,(x,y),(x+w,y+h),(255,0,0),2)
            face_gray = np.array(im_gray[y:y+h, x:x+w], 'uint8')
            face_sized = cv2.resize(face_gray, (200, 200))
            face_images.append(face_sized)
            face_labels.append(face_label)
            facecount += 1
      cv2.imshow("Training Set Preparation", im)
      cv2.waitKey(3)
      rate.sleep()

    cv2.destroyWindow("Training Set Preparation")
    return face_images, face_labels

  def recognize_faces(self, names):

    height = 480
    width = 640
    rate = rospy.Rate(10)
    cv2.namedWindow("Face Recognizer", 1)

    while not rospy.is_shutdown():
      if(self.is_image_present):
        im = self.image
      else:
        im = np.ones((height,width,3), np.uint8)
      (rows,cols,channels) = im.shape
      im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
      faces = face_cascade.detectMultiScale(im_gray, 1.3, 5)
      box = all_faces()
      for (x,y,w,h) in faces:
        if ((w>50) and (h>50)):
          cv2.rectangle(im,(x,y),(x+w,y+h),(255,0,0),2)
          face_gray = np.array(im_gray[y:y+h, x:x+w], 'uint8')
          face_sized = cv2.resize(face_gray, (200, 200))
          face_predicted, conf = recognizer_lb.predict(face_sized)
          print 'This is ', names[face_predicted],
          print '| Confidence: ', conf 
          box.x.append(x)       
          box.y.append(y)       
          box.w.append(w)       
          box.h.append(h)       
          box.s.append(names[face_predicted])
      if(len(box.s) > 0):
        self.f_publ.publish(box)

      cv2.imshow("Face Recognizer", im)
      cv2.waitKey(3)
      rate.sleep()

def main(args):
  ic = face_node()
  rospy.init_node('face_node', anonymous=True)
  
  count = 0
  label_names = []
  
  print 'Training set size: ', TRAIN_SIZE
  ans = raw_input('Train faces? ')
  if (ans == 'yes'):
    name = raw_input('Name of the person: ')
    train_faces, train_labels = ic.prepare_face_data(count);
    label_names.append(name)
    
    while(ans == 'yes'):
      ans = raw_input('Train the model with another person? ')
      if (ans == 'yes'):
        count += 1
        name = raw_input('Name of the person: ')
        train_faces_new, train_labels_new = ic.prepare_face_data(count);
        label_names.append(name)
        train_faces = train_faces + train_faces_new
        train_labels = train_labels + train_labels_new
    
    # added: restore previous data
    if os.path.isfile(PATH+'facedata.pkl') and os.access(PATH+'facedata.pkl', os.R_OK):
      choice = raw_input('Load existing face data? ')
      if (choice == 'yes'):
        with open(PATH+'facedata.pkl', "rb") as data:
          faces0, labels0, names0 = pkl.load(data)
          label_names.extend(names0)
          train_faces = train_faces + faces0
          train_labels = train_labels + labels0
    
    choice = raw_input('Overwrite face data? If you do not, this run will be ignored: ')
    if (choice == 'yes'):
      print 'Storing dataset...'
      data = [train_faces, train_labels, label_names]          
      with open(PATH+'facedata.pkl', 'wb') as output:
        pkl.dump(data, output)

  print 'Preparing dataset...'
  with open(PATH+'facedata.pkl', "rb") as data:
    datasets = pkl.load(data)
  faces, labels, names = datasets
  recognizer_lb.train(faces, np.array(labels))

  ic.recognize_faces(names)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
