cmake_minimum_required(VERSION 2.8.3)
project(bender_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  baseArmState.msg
  baseEfectorPosition.msg
  BlobList.msg
  CartesianInfo.msg
  Command.msg
  data_follow.msg
  Emotion.msg
  FKinfo.msg
  grip.msg
  KinectTrackerData.msg
  OclusionData.msg
  PathBlockingDetections.msg
  PlaneDetection.msg
  PoseDetections.msg
  Rect.msg
  ROIArray.msg
  SemanticObject.msg
  SiftDetection.msg
  SiftDetectionDepth.msg
  SiftDetectionFloor.msg
  StateInfo.msg
  WaveData.msg
  EmotionDetect.msg
  detections.msg
  CylindricalObject.msg
  Marker.msg 
  MarkerArray.msg 
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
