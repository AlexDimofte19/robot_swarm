# A simple differential drive robot with a laser range finder

define laser ranger
(
  sensor
    (
      range [ 0.0  3.0 ]
      fov 360
      samples 720
    )

    color "black"
    size [ 0.05 0.05 0.1 ]
)

define robot position
(
  size [0.1 0.1 0.1]
  laser(pose [0 0 0.0 0])
  drive "diff"
)

define robot_omni position
(
  size [0.1 0.1 0.1]
  laser(pose [0 0 0 0])
  drive "omni"
)