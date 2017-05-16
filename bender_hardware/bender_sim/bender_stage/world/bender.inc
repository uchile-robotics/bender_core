
include "../hokuyos.inc"

# bender
define bender position
(

  color "red"        # Default color.
  drive "diff"       # Differential steering model.
  gui_nose 1         # Draw a nose so we can see which way it points
  obstacle_return 1  # Can hit things.
  
  #localization "gps"             
  #localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"            # Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]  # Odometry error or slip in X, Y and Theta
                                   # (Uniform random distribution)   

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]        
  #velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]          
  #acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]

  # estimated mass in KG
  #mass 23.0 

  size [0.500 0.400 0.300]
  origin [-0.050 0.000 0.000 0.000]


  # bender_footprint
  block(
    points 13
    point[0] [0.25 0.22]
    point[1] [0.27 0]
    point[2] [0.25 -0.22]
    point[3] [0.07 -0.22]
    point[4] [0.07 -0.34]
    point[5] [-0.07 -0.34]
    point[6] [-0.07 -0.22]
    point[7] [-0.27 -0.22]
    point[8] [-0.27 0.22]
    point[9] [-0.07 0.22]
    point[10] [-0.07 0.34]
    point[11] [0.07 0.34]
    point[12] [0.07 0.22]
    z [0 0.1]
  )
  
  # laser front
  hokuyo_front()

  # laser rear
  hokuyo_rear()

)
