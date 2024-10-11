# This is a "Human" robot, it represents to legs, it can move omni-directional (both x and y, and rotate)

define leg model
(
  size [0.1 0.1 0.3]
  block
  (
    points 18
    point[0] [1.0000 0.0000]
    point[1] [0.9397 0.3420]
    point[2] [0.7660 0.6428]
    point[3] [0.5000 0.8660]
    point[4] [0.1736 0.9848]
    point[5] [-0.1736 0.9848]
    point[6] [-0.5000 0.8660]
    point[7] [-0.7660 0.6428]
    point[8] [-0.9397 0.3420]
    point[9] [-1.0000 0.0000]
    point[10] [-0.9397 -0.3420]
    point[11] [-0.7660 -0.6428]
    point[12] [-0.5000 -0.8660]
    point[13] [-0.1736 -0.9848]
    point[14] [0.1736 -0.9848]
    point[15] [0.5000 -0.8660]
    point[16] [0.7660 -0.6428]
    point[17] [0.9397 -0.3420]
    z [0 0.1]
  )

  laser_return 1
  obstacle_return 1
)

define human position
(
  size [0.05 0.2 0.1]
  origin [0 0 0.3 0]
  leg(pose [0 0.1 -0.1 0] color "red")
  leg(pose [0 -0.1 -0.1 0] color "red")
  
  obstacle_return 1
  drive "omni"
)