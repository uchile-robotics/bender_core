
# hokuyo urg-04lx-ug01 (The Black One!)
define hokuyo_urg_04lx_ug01 ranger
(
  # all data from datasheet at:
  # http://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx_ug01.html

  model
  (
    color "black"
    size [ 0.050 0.050 0.070 ]
  )
)

# hokuyo utm-30LX (The Orange One!)
define hokuyo_umt_30lx ranger
(
  # all data from datasheet at:
  # https://www.hokuyo-aut.jp/02sensor/07scanner/utm_30lx.html

  model
  (
    color "orange"
    size [ 0.060 0.060 0.087 ]
  )
)

define hokuyo_rear hokuyo_urg_04lx_ug01
(
  sensor(
    range [ 0.0  5.6 ]  # [m]
    fov     160.0       # max fov 240° -  ~682 samples
    samples 640         # 360°/1,024 steps 
  )

  pose [ -0.220 0.000 0.009 180.000 ]
)

define hokuyo_front hokuyo_umt_30lx
(
  sensor(
    range [ 0.0 30.0 ]  # [m]   
    #fov     180.0       # max fov 270° - 1080 samples
    #samples 720         # 360°/1,440 steps 
    fov     230.0       # max fov 270° - 1080 samples
    samples 920        # 360°/1,440 steps 
  )

  pose [ 0.220 0.000 0.033 0.000 ]
)