cmake_minimum_required(VERSION 2.8.3)
project(uchile_msgs)

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
  CylindricalObject.msg
  data_follow.msg
  detections.msg
  FaceEmotion.msg
  EmotionDetect.msg
  FKinfo.msg
  grip.msg
  KinectTrackerData.msg
  LedCommand.msg 
  LedCommand2.msg 
  Marker.msg 
  MarkerArray.msg 
  Num.msg 
  ObjectDetection.msg
  OclusionData.msg
  PathBlockingDetections.msg
  PlaneDetection.msg
  PoseDetections.msg
  Rect.msg
  ROIArray.msg
  SemanticObject.msg
  ServoCommand.msg 
  ServoState.msg 
  SiftDetection.msg
  SiftDetectionDepth.msg
  SiftDetectionFloor.msg
  StateInfo.msg
  WaveData.msg
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
