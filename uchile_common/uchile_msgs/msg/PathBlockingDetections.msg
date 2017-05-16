
std_msgs/Header header

# detections
geometry_msgs/Pose[] detections

# detector specific extra information
string[] info

# advise from the detector to take an specific action
bool should_take_action