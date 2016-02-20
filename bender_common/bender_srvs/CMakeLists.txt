cmake_minimum_required(VERSION 2.8.3)
project(bender_srvs)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
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
  ArmCommand.srv
  Bool.srv
  AngVel.srv
  NavGoal.srv
  Detection.srv
  setAnglesBaseArm.srv
  armGoal.srv
  DetectState.srv
  ObjectDetection2.srv
  Recognition.srv
  setGoalBaseArm.srv
  ArmState.srv
  PoseStampedArray.srv
  ObjectDetection.srv
  ObjectManipulation.srv
  siftClearDatabase.srv
  AskQuestion.srv
  objrecService.srv
  siftFileTrain.srv
  BlobList.srv
  Onoff.srv
  GripperInfo.srv
  siftGuiTrain.srv
  DetectEmerg.srv
  orientBaseArm.srv
  DoorDetector.srv
  PlaneHeight.srv
  siftOnlineTrain.srv
  Dummy.srv
  FaceInfo.srv
  Slope.srv
  FaceRecognition.srv
  planeXY.srv
  Speed.srv
  posXYZ.srv
  Feedback.srv
  PlanIKGoal.srv
  State.srv
  Float.srv
  PlanningGoalCartesian.srv
  States.srv
  getLaserScan.srv
  PlanningGoalState.srv
  stringReturn.srv
  intInfo.srv
  ID.srv
  play_sound.srv
  stringService.srv
  ImageService.srv
  PoseStamped.srv
  String.srv
  IsOn.srv
  Punch.srv
  synthesize.srv
  load_dictionary_service.srv
  QuestionService.srv
  TableDetector.srv
  LoadMode.srv
  ReportGenerator.srv
  TorqueEnable.srv
  manipBaseArm.srv
  SearchPerson.srv
  TorqueLimit.srv
  Margin.srv
  selectBrazo.srv
  Transformer.srv
  MaskInfo.srv
  SemMap.srv
  moveRosaria.srv
  sendPoseEstimation.srv
  ValidPoint.srv
  PointCloud2.srv
  PointCloud2Trasnform.srv
  ImageDetection.srv
  GenderDetection.srv
  TimerString.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  bender_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package()

