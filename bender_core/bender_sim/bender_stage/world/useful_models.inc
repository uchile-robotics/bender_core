
# - - - - Map - - - -

# map
define map model
(
  # sombre, sensible, artistic
  color "gray30"

  # no bounding box
  boundary 0  

  gui_nose 0
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
  #ranger_return 0.500  # reflects sonar beams
  #blob_return 1        # Seen by blobfinders  
)

# - - - - Person - - - -

define person model
(
  gui_nose 1
  color "orange"

  # simplified Body shape:
  block( 
    points 11
    point[0] [-0.07 0.35]
    point[1] [-0.15 0.25]
    point[2] [-0.15 -0.25]
    point[3] [-0.07 -0.35]
    point[4] [0.07 -0.35]
    point[5] [0.15 -0.25]
    point[6] [0.15 -0.15]
    point[7] [0.20 0.0]
    point[8] [0.15 0.15]
    point[9] [0.15 0.25]
    point[10] [0.07 0.35]
    z [0.0 0.22]
  )

  size [0.300 0.300 1.800]
)


# - - - - Other - - - -

define branch model
(
  size [1.000 0.100 0.800]
  gui_nose 1
  color "brown"
)

# - - - - legs - - - -

define legMedium model
(
  size [0.150 0.150 0.800]
  gui_nose 1
  color "blue"
)

define legSmall model
(
  size [0.100 0.100 0.800]
  gui_nose 1
  color "green"
)
