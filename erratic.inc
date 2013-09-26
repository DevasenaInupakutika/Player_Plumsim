# Desc: Device definitions for Erratic robots.
# Author: Kurt Konolige
# Date: October 2006


# Erratic ERA platform, Rev E
define erratic position
(
  # actual size
  size [0.44 0.33 0.3]

  # the center of rotation is offset from its center of area
  origin [-0.08 0.0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 12.0 
	
  # this polygon approximates the shape of erratic
  polygons 1
  polygon[0].points 8
  polygon[0].point[0] [  0.19  0.11 ]
  polygon[0].point[1] [  0.13  0.19 ]
  polygon[0].point[2] [ -0.13  0.19 ]
  polygon[0].point[3] [ -0.19  0.10 ]
  polygon[0].point[4] [ -0.19 -0.10 ]
  polygon[0].point[5] [ -0.13 -0.19 ]
  polygon[0].point[6] [  0.13 -0.19 ]
  polygon[0].point[7] [  0.19 -0.11 ]

  # differential steering model
  drive "diff"

  # uncomment this line if you want to model real Erratics with SICK
  # lasers, where the laser is taller than the robot
  # laser_return 0

  laser_return 1
  obstacle_return 1
  
  #localization "odom"     #change to "gps" impossibly perfect, global odometry
  #localization_origin [0 0 0] #Start odometry at (0,0,0)
  #odom_error [0.01 0.01 0.020] #Odometry error
) 

#define erratic_battery erratic
#(
#  power( pose [-0.14 0 0] capacity 1000 probe_range 0.50  give 0 give_rate 100 take_rate 100 )
#)

#
# no model for IR floor sensors
#


#
# Erratic front sonar array
#
define erratic_sonar ranger
(
  scount 8

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.180 0.110 90 ]
  spose[1] [ 0.200 0.085  0 ]
  spose[2] [ 0.190 0.065 -53]
  spose[3] [ 0.170 0.045 -24]
  spose[7] [ 0.170 -0.045 24]
  spose[6] [ 0.190 -0.065 53]
  spose[5] [ 0.200 -0.085  0]
  spose[4] [ 0.180 -0.110 -90]
		
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 5.0 20]

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.01 0.05]
)

