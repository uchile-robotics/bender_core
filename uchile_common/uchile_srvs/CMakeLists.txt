cmake_minimum_required(VERSION 2.8.3)
project(uchile_srvs)

find_package(catkin REQUIRED COMPONENTS
  uchile_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################



# Generate services in the 'srv' folder
add_service_files(
  FILES
  Bool.srv
  String.srv
  Float.srv
  stringReturn.srv
  stringService.srv
  ID.srv
  Onoff.srv
  IsOn.srv
  PoseStamped.srv
  NavGoal.srv
  Detection.srv
  DetectState.srv
  Recognition.srv
  ObjectDetection.srv
  siftClearDatabase.srv
  objrecService.srv
  siftFileTrain.srv
  BlobList.srv
  siftGuiTrain.srv
  DetectEmerg.srv
  DoorDetector.srv
  PlaneHeight.srv
  siftOnlineTrain.srv
  ImageDetector.srv
  FaceRecognition.srv
  play_sound.srv
  ImageService.srv
  synthesize.srv
  load_dictionary_service.srv
  TableDetector.srv
  ReportGenerator.srv
  SearchPerson.srv
  Transformer.srv
  MaskInfo.srv
  SemMap.srv
  ValidPoint.srv
  PointCloud2.srv
  PointCloud2Trasnform.srv
  ImageDetection.srv
  GenderDetection.srv
  TimerString.srv
  ImageRecognition.srv
  ImageReport.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  uchile_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package()
