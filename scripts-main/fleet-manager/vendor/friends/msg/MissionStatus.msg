# This message holds the status of mission shared by mission modules via /mission_status topic

# Time of sensor data acquisition and coordinate frame
Header header

# string unique of the drone sharing the msg
string drone_id

# mission-module that is running: Scouting, Monitoring, Inspection, Pathfollowing
# only first letter is capital.
string task

# type of message: Error, Info, Progress, State
string type 

# message being shared
string message
