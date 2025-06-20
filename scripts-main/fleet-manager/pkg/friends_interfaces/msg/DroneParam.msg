#all params necessary to define a mission
string drone_id
string dtype
float32 dmax_speed
float32 dobst_dist
float32 dmin_bat
float32[2] dhome

float32 dsize

# comments by order:
#A drone needs name and description when added.
#Set maximum flight speed.
#Minimum distance to obstacles allowed while flying 
#Minimum value for bat percentage (allowed for flight).
#Mission enters emergency mode if bat below this value
#Define,the home location for the drone.
#When adding a new drone , define which drone it is  from a fixed list. This defines which sensor package is encompassed.
#Minimum diameter for the sphere that can circumscribe the drone.




