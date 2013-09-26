
# The roomba sonar array
define roomba_sonar ranger
(
  scount 5 		# the number of transducers

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.0 0.15 90 ]
  spose[1] [ 0.105 0.105 45 ]
  spose[2] [ 0.15 0.0 0 ]
  spose[3] [ 0.105 -0.105 -45 ]
  spose[4] [ 0.0 -0.15 -90 ]
  	
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 1.0 30]

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.02 0.02]
)

# roomba robot
define roomba position
(
  size [0.33 0.33 0.1]

  # this block approximates the circular shape of a Roomba
  block( 
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    
 #   color "gray50"
  )
  
  # bumper_1 case
  block
  (
    points 3
    point[0]  [ 0.200 0.096 ]
    point[1]  [ 0.200 0.066 ]
    point[2]  [ 0.190 0.080 ]
    #z [ 0.2 0.2 ]
    color "red"
  )

  # bumper_2 case
  block
  (
    points 3
    point[0]  [ 0.200 -0.096 ]
    point[1]  [ 0.200 -0.066 ]
    point[2]  [ 0.190 -0.080 ]
    #z [ 0.2 0.2 ]
    color "red"
  )
  
 
   # this bumper array VERY crudely approximates the Roomba's bumpers
#  bumper
#  ( 
#		bcount 2  
#	  	blength 0.33
#        bpose[0] [0.12  0.12  45]
#        bpose[1] [0.12 -0.12 -45] 
#  )
 
  
  roomba_sonar( pose [0 0 0.1 0] ) 
   
  
  color "gray50"
)

define create roomba( color "gray90" )

# background object bg
define bg position
(
  size [0.10 0.10 0.10]
  color "blue"
)

define create bg( color "blue" )

