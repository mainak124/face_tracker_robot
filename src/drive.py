#!/usr/bin/env python
import struct
import sys, glob # for listing serial ports
import roslib
roslib.load_manifest('team_3_robot_follower')
import sys, os, os.path
import rospy
import numpy as np
from team_3_robot_follower.msg import tracking

try:
    import serial
except ImportError:
    print 'Import error', 'Please install pyserial.'
    raise

connection = None

TEXTWIDTH = 40 # window width, in characters
TEXTHEIGHT = 16 # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

x0 = 0.
y0 = 0.
x1 = 0.
y1 = 0.
cb_started = 0

class drive:

  callbackKeyLastDriveCommand = ''

  # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
  def sendCommandASCII(self, command):
      cmd = ""
      for v in command.split():
          cmd += chr(int(v))

      self.sendCommandRaw(cmd)

  # sendCommandRaw takes a string interpreted as a byte array
  def sendCommandRaw(self, command):
    global connection

    try:
      if connection is not None:
        connection.write(command)
      else:
        print "Not connected."
    except serial.SerialException:
      print "Lost connection"
      connection = None

    # print ' '.join([ str(ord(c)) for c in command ])

  def rotate(self, velocity, rate, angle):
    count = 0
    #velocity = 0
    abs_angle = np.abs(angle)
    while not rospy.is_shutdown() and (count < 1):
      if (abs_angle < 10): # 20 Degree
        rotation = 0
      elif (abs_angle >= 10) and (abs_angle < 30): # 20 Degree
        rotation = 100 if (angle > 0) else -100
      elif (abs_angle >= 30) and (abs_angle < 50): # 40 Degree
        rotation = 800 if (angle > 0) else -800
      elif (abs_angle >= 50) and (abs_angle < 70): # 60 Degree
        rotation = 1200 if (angle > 0) else -1200
      elif (abs_angle >= 70): # and (abs_angle < 70): # 80 Degree
        rotation = 1600 if (angle > 0) else -1600
      self.move(velocity, rotation)
      print "self.move(", velocity, rotation,")"
      count += 1
      rate.sleep()
    #rotation = 0
    #self.move(velocity, rotation)
    #rate.sleep()

  def movement(self, rate, pace, is_plus, angle):
    count = 0
    #while not rospy.is_shutdown() and (count < 1):
    if (pace == 'fast'):
      velocity = 500 if (is_plus==1) else -500
    elif (pace == 'medium'):
      velocity = 250 if (is_plus==1) else -250
    elif (pace == 'slow'):
      velocity = 100 if (is_plus==1) else -100
    elif (pace == 'zero'):
      velocity = 0
    self.rotate(velocity, rate, angle)
    # rotation = 0
    # self.move(velocity, rotation)
    # count += 1
    # rate.sleep()

  def move(self, velocity, rotation):

    # compute left and right wheel velocities
    vr = velocity + (rotation/2)
    vl = velocity - (rotation/2)

    # create drive command
    cmd = struct.pack(">Bhh", 145, vr, vl)
    #if cmd != self.callbackKeyLastDriveCommand:
    self.sendCommandRaw(cmd)
    self.callbackKeyLastDriveCommand = cmd

  def selfDrive(self):

    self.onConnect()
    self.sendCommandASCII('128')
    self.sendCommandASCII('131')
    self.sendCommandASCII('128')
    self.sendCommandASCII('131')

    rate = rospy.Rate(5)
    count = 0

    prev_x0 = 0.
    prev_y0 = 0.
    prev_x1 = 0.
    prev_y1 = 0.
    innerloop = 0

    while not rospy.is_shutdown():
      innerloop = 0
      if (cb_started == 1):
        actual_dist = np.sqrt(np.square(x0) + np.square(y0))
        actual_angle = np.arctan(y0/x0)*(180/np.pi)
        pred_angle = np.arctan(y1/x1)*(180/np.pi)
        print 'dist', int(actual_dist*10), 'pred_angle', int(pred_angle),
        innerloop = 0

        if (x0<1.3): # move back
            if(x1<x0): # coming closer
                if (1.9 - x1) > 0.5: # move back fast
                    self.movement(rate, 'fast', 0, pred_angle)
                    print "condition 1"
                else:
                    self.movement(rate, 'medium', 0, pred_angle)
                    print "condition 2"
            else: # going further
                if (1.9 - x0) > 0.5: # move back medium
                    self.movement(rate, 'medium', 0, pred_angle)
                    print "condition 3"
                else:
                    self.movement(rate, 'slow', 0, pred_angle)
                    print "condition 4"
            innerloop = 1
            print '[back]',
        elif (x0>1.5):
            if(x1>x0): # going further
                if (x1 - 1.9) > 0.5: # move forward fast
                    self.movement(rate, 'fast', 1, pred_angle)
                    print "condition 5"
                else:
                    self.movement(rate, 'medium', 1, pred_angle)
                    print "condition 6"
            else: # coming closer
                if (x0 - 1.9) > 0.5: # move forward medium
                    self.movement(rate, 'medium', 1, pred_angle)
                    print "condition 7"
                else:
                    self.movement(rate, 'slow', 1, pred_angle)
                    print "condition 8"
            print '[forward]',
            innerloop = 1
        else:
            self.movement(rate, 'zero', 1, pred_angle)
            print "condition 9"
            innerloop = 1

        # if (count < 5):
        #    #self.movement(rate, 'fast', 0, pred_angle)
        #    self.rotate(0, rate, 10)
        #    innerloop = 1

        count += 1
        if (innerloop == 0):
            rotation = 0
            velocity = 0
            self.move(velocity, rotation)
            rate.sleep()
      #if (innerloop == 0):
      #    rate.sleep()

  def onConnect(self):
    global connection

    if connection is not None:
      print 'Oops Youre already connected!'
      return

    port = '/dev/ttyUSB0'
    if port is not None:
      print "Trying " + str(port) + "... "
      try:
        connection = serial.Serial(port, baudrate=115200, timeout=1)
        print "Connected!"
      except:
        print "Failed."

def l_cb(data):
    #print 'x:', data.x0, data.x1, '| y:', data.y0, data.y1 
    global x0
    global y0
    global x1
    global y1
    global cb_started
    x0 = data.x0
    y0 = data.y0
    x1 = data.x1
    y1 = data.y1
    cb_started = 1


def main(args):
  drive_bot = drive()
  rospy.init_node('drive', anonymous=True)
  rospy.Subscriber('/tracking', tracking, l_cb)
  drive_bot.selfDrive()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("shutting down")

if __name__ == '__main__':
  main(sys.argv)
