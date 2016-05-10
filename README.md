# face_tracker_robot
Face recognition based human tracking using i-Robot in ROS

This system first finds the faces in the view, identifies whom to track by recognizing the face (It should be trained in a supervised manner with that person's face); using Kinect's depth sesnsor it registers the head and then tracks the head using Kalman Filter.
