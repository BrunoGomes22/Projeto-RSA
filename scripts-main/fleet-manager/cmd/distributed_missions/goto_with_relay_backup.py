import argparse
import Math

from time import sleep
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading

# Global variables
myDroneId = ''
publisherCmd = ''
publisherInfo = ''
GS_coords = (40.63418785269594, -8.660018532516903)
onMission = False
withRelay = False
lock = threading.Lock()

# Status subscribers
currentStatus = None
status_subscriber = None

# Telem subscribers
currentTelem = None

# Cmd subscribers
currentCmd = None
cmd_subscriber = None

# Info subscribers
currentInfo = None
info_subscriber = None

# Distance to keep between nodes
target_dist = 100 # meters

# Error accepted in calculations
tolerance = 2 # meters

def current_time_millis():
    return int(round(time.time() * 1000))

def base_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    return {'droneId' : droneId, 'mode' : 'action', 'timestamp' : current_time_millis()}

def publish_cmd(cmd):
    global publisherCmd

    msg = String()    
    msg.data = json.dumps(cmd)
    publisherCmd.publish(msg)

def publish_info(info):
    global publisherInfo

    msg = String()    
    msg.data = json.dumps(info)
    publisherInfo.publish(msg)

def arm_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    cmd = base_cmd(droneId=droneId)
    cmd['cmd'] = 'arm'
    publish_cmd(cmd)

def takeoff_cmd(droneId=None,alt=10):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    cmd = base_cmd(droneId=droneId)
    cmd['cmd'] = 'arm'
    cmd['alt'] = alt
    publish_cmd(cmd)
    sleep(1)
    cmd['cmd'] = 'takeoff'
    publish_cmd(cmd)

def land_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    cmd = base_cmd(droneId=droneId)
    cmd['cmd'] = 'land'
    publish_cmd(cmd)

def goto_cmd(lat, lon, alt=None, yaw=None, speed=None,droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    cmd = base_cmd(droneId=droneId)
    cmd['cmd'] = 'goto'
    cmd['lat'] = lat
    cmd['lon'] = lon
    if alt != None:
        cmd['alt'] = alt    
    cmd['yaw'] = yaw
    if speed != None:
        cmd['speed'] = speed
    publish_cmd(cmd)

def cancel_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    cmd = base_cmd(droneId=droneId)
    cmd['mode'] = 'custom'
    cmd['cmd'] = 'cancel'
    publish_cmd(cmd)

class TelemSubscriber(Node):     
    def __init__(self):
        global myDroneId
        name = 'telem_subscriber_' + myDroneId

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/telem',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        global currentTelem
                         
        currentTelem = msg.data     

class StatusSubscriber(Node):    
    def __init__(self):
        global myDroneId
        name = 'status_subscriber_' + myDroneId

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        global currentStatus
                         
        currentStatus = msg.data     

class CmdSubscriber(Node):    
    def __init__(self):
        global myDroneId
        name = 'cmd_subscriber_' + myDroneId

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        global currentCmd
                         
        currentCmd = msg.data  
 
class InfoSubscriber(Node):    
    def __init__(self):
        global myDroneId
        name = 'info_subscriber_' + myDroneId

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        global currentInfo
                         
        currentInfo = msg.data  

def waitForTask(cmd=None,state=None,droneId=None):    
    global currentStatus
    global myDroneId
    global status_subscriber
    global lock

    if droneId == None:
        droneId = myDroneId

    while True:   
        lock.acquire()   
        rclpy.spin_once(status_subscriber, timeout_sec=1)  
        lock.release()  
        tmp = json.loads(currentStatus)  
        if 'command' in tmp:      
            if tmp['droneId'] == droneId and tmp['command'] == cmd and tmp['state'] == state :           
                break        

def checkForRelay():
    global currentCmd
    global currentInfo
    global myDroneId
    global cmd_subscriber
    global info_subscriber
    global GS_coords
    global target_dist
    global onMission
    global lock

    while True:   
        # If the mission is over, return  
        if not onMission:
            return
        # Check for position command
        lock.acquire()   
        rclpy.spin_once(cmd_subscriber, timeout_sec=1)  
        lock.release()  
        tmp = json.loads(currentCmd)  
        # Check if the cmd message is from itself
        if tmp['droneId'] == myDroneId and tmp['cmd'] == 'goto': 
            Lat = tmp['lat']
            Lon = tmp['lon']    
            # Check if the distance for the next target coordinate is too far
            # and if it'll need a relay drone      
            if Math.requiresRelay(GS_coords,(Lat,Lon),target_dist):
                print("NEEDS RELAY!")
                msg = {'TO' : 'GroundStation',
                       'FROM' : myDroneId,
                       'TYPE' : 'requestRelay'}
                publish_info(msg)                

                while True:
                    # Send relay request to GroundStation
                    lock.acquire()   
                    rclpy.spin_once(info_subscriber, timeout_sec=1)  
                    lock.release()  
                    tmp = json.loads(currentInfo)  
                    print("tmp =",tmp)
                    if tmp['FROM'] == 'GroundStation' and tmp['TO'] == myDroneId and tmp['TYPE'] == 'relayAccepted':
                        print("relayAccepted")
                        return
                    else:
                        # Resend message ?
                        pass

            # If the request is not needed
            else:
                print("Doesn't need relay!")
                pass

#def checkForCancel():

    
def main(args=None):
    global myDroneId
    global publisherCmd
    global publisherInfo
    global status_subscriber 
    global cmd_subscriber
    global info_subscriber
    global onMission

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', required=True)
    cli_args = parser.parse_args()

    myDroneId = cli_args.drone

    rclpy.init(args=args)

    # Initialize publishers
    node = rclpy.create_node('publisher' + myDroneId)
    publisherCmd = node.create_publisher(String, '/cmd', 10)
    publisherInfo = node.create_publisher(String, '/info', 10)

    # Initialize subscribers
    status_subscriber = StatusSubscriber() 
    cmd_subscriber = CmdSubscriber()
    info_subscriber = InfoSubscriber()

    # To make sure every message is sent
    sleep(5)

    onMission = True

    # Takeoff
    takeoff_cmd()
    waitForTask(cmd='takeoff',state='finish') 

    # Threads used for check for relays
    checkRelayThread = threading.Thread(target=checkForRelay)
    checkRelayThread.start()

    goto_cmd(40.6350657883063, -8.660733645429573, alt=20)
    waitForTask(cmd='goto',state='finish')

    goto_cmd(40.635444016098305, -8.65869124034782, alt=20)
    waitForTask(cmd='goto',state='finish')

    goto_cmd(40.6339939208923, -8.657955785830225, alt=20)
    waitForTask(cmd='goto',state='finish')

    goto_cmd(40.632731660151265, -8.658704576895918, alt=25)
    waitForTask(cmd='goto',state='finish')

    goto_cmd(40.632537130900864, -8.66022239456038, alt=25)
    waitForTask(cmd='goto',state='finish')

    goto_cmd(40.63351294914651, -8.662097348147393, alt=25)
    waitForTask(cmd='goto',state='finish')

    land_cmd()
    waitForTask(cmd='land',state='finish') 

    onMission = False

    checkRelayThread.join()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    
    # Destroy main node and shutdown
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()