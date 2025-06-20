# This message holds the status of mission shared by mission modules via /mission_status topic

# Time of sensor data acquisition and coordinate frame
Header header

# string unique of the drone sharing the msg
string mode

# wapoints
float32[] waypoints
