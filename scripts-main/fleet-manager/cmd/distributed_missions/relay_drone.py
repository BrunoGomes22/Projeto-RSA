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
droneToFollow = None  
droneFollowingMe = None
publisherInfo = ''
publisherCmd = ''
onMission = False
GS_coords = (40.63418785269594, -8.660018532516903)
lock = threading.Lock()

# Distance to keep between nodes
target_dist = 65 # meters

# Error accepted in calculations
tolerance = 2 # meters

#Status subscriber
currentStatus = None
status_subscriber = None

#Telem subscriber
telem_subscriber = None
currentTelem = None
TelemOrigin = None
TelemTarget = None

#Cmd subscriber
cmd_subscriber = None
currentCmd = None

# Info subscribers
currentInfo = None
info_subscriber = None

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

def return_home_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    cmd = base_cmd(droneId=droneId)
    cmd['cmd'] = 'return'
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
        rclpy.spin_once(status_subscriber, timeout_sec=0.5)  
        tmp = json.loads(currentStatus) 
        lock.release() 
        if 'command' in tmp:      
            if tmp['droneId'] == droneId and tmp['command'] == cmd and tmp['state'] == state :           
                break   

def waitForRelayRequest():
    global currentInfo
    global info_subscriber

    while True:
        rclpy.spin_once(info_subscriber)
        tmp = json.loads(currentInfo) 
        if tmp['FROM'] == 'GroundStation' and tmp['TO'] == myDroneId and tmp['TYPE'] == 'relayAssigned':
            return tmp['droneAssigned']

def relayCalc(originDrone=None,targetDrone=None):
    # originDrone = Drone running this code
    # targetDrone = Drone this drone was assigned to follow  
    global GS_coords
    global onMission
    global currentTelem
    global telem_subscriber
    global myDroneId
    global target_dist
    global tolerance
    global lock

    if originDrone == None:
        originDrone = myDroneId

    # Variables to save
    targetLat = 0
    targetLon = 0
    targetAlt = 0   

    # Wait for goto coordinate from target drone
    while True:
        lock.acquire()
        rclpy.spin_once(telem_subscriber, timeout_sec=0.5)
        tmp = json.loads(currentTelem) 
        lock.release()
        if tmp['droneId'] == targetDrone:
            targetLat = tmp['position']['lat']
            targetLon = tmp['position']['lon']
            targetAlt = tmp['position']['alt']
            targetSpeed = tmp['speed']
            break     

    target_info = (targetLat,targetLon,targetAlt,targetSpeed)

    return Math.relayCalc(GS_coords,target_info,target_dist,tolerance)    

def checkIfRelayIsNeeded(Lat,Lon):
    global currentInfo
    global myDroneId
    global info_subscriber
    global GS_coords
    global target_dist
    global onMission
    global droneFollowingMe
    global droneToFollow
    
    #print("checkIfRelayIsNeeded()")
    # If the drone has no drones following him
    if droneFollowingMe == None:
        # Check if the distance for the next target coordinate is too far
        # and if it'll need a relay drone      
        if Math.requiresRelay(GS_coords,(Lat,Lon),target_dist):
            msg = {'FROM' : myDroneId,
                'TO' : 'GroundStation',
                'TYPE' : 'requestRelay'}
            # Send message
            publish_info(msg)                
            while True:
                # Send relay request to GroundStation
                lock.acquire()   
                rclpy.spin_once(info_subscriber, timeout_sec=0.5)  
                tmp = json.loads(currentInfo) 
                lock.release()  
                if tmp['FROM'] == 'GroundStation' and tmp['TO'] == myDroneId:
                    if tmp['TYPE'] == 'relayAccepted':
                        droneFollowingMe = tmp['droneAssigned']
                    elif tmp['TYPE'] == 'relayDenied':
                        onMission = False
                        # Inform drone that it's following that the 
                        # relay has been cancelled
                        msg = {'FROM' : myDroneId,
                        'TO' : droneToFollow,
                        'TYPE' : 'relayDenied'}
                        publish_info(msg) 
                        #sleep(0.5)
                        '''msg = {'FROM' : myDroneId,
                        'TO' : 'GroundStation',
                        'TYPE' : 'freeDrone'}
                        publish_info(msg)'''
                    return
                else:
                    # Resend message ?
                    pass
    # If there's a drone following him
    else:
        # In case the relay needs to be cancelled
        if not Math.requiresRelay(GS_coords,(Lat,Lon),target_dist):
            # Send message to GS to inform he's not
            # being followed anymore
            msg = {'FROM' : myDroneId,
                'TO' : 'GroundStation',
                'TYPE' : 'cancelRelay',
                'droneAssigned' : droneFollowingMe}
            publish_info(msg) 
            #sleep(0.5) 
            # Send message to the drone that is following
            # that it can return home
            msg = {'FROM' : myDroneId,
            'TO' : droneFollowingMe,
            'TYPE' : 'freeDrone'}
            publish_info(msg) 
            droneFollowingMe = None     

def goto(Lat, Lon, alt=None, speed=None):
    global droneFollowingMe
    global onMission

    if onMission:
        checkIfRelayIsNeeded(Lat,Lon)
        goto_cmd(Lat,Lon,alt=alt,speed=speed)
      
def relayHandler(): 
    global droneToFollow
    global droneFollowingMe
    global onMission

    # Thread to check if relay has been cancelled
    checkCancel = threading.Thread(target=checkForRelayCancelOrMissionEnd)
    checkCancel.start()

    while onMission:
        print("relayHandler()")
        targetLat, targetLon, targetAlt, targetSpeed = relayCalc(targetDrone=droneToFollow)
        if targetLat == None:
            cancel_cmd()
        else:          
            goto(targetLat, targetLon, alt=targetAlt, speed=12)
        # Free thread
        sleep(0.01)

    print("BEFORE checkCancel.join()")
    checkCancel.join()  
    droneFollowingMe = None  
    return_home_cmd()

def checkForRelayCancelOrMissionEnd():
    global myDroneId
    global info_subscriber
    global onMission
    global droneFollowingMe
    global droneToFollow
    global lock

    while onMission:
        lock.acquire()  
        print("checkForRelayCancelOrMissionEnd()") 
        rclpy.spin_once(info_subscriber, timeout_sec=0.5)  
        tmp = json.loads(currentInfo)
        lock.release() 
        if tmp['TO'] == myDroneId: 
            # In case it receives a freeDrone message
            if tmp['TYPE'] == 'freeDrone':
                if droneFollowingMe != None:
                    # Send message to the drone that is following
                    # that it can return home
                    msg = {'FROM' : myDroneId,
                    'TO' : droneFollowingMe,
                    'TYPE' : 'freeDrone'}
                    publish_info(msg) 
                    droneFollowingMe = None  
                    #sleep(0.5)

                    # Send message to GS to inform he's not
                    # being followed anymore
                    '''msg = {'FROM' : myDroneId,
                        'TO' : 'GroundStation',
                        'TYPE' : 'cancelRelay'}
                    publish_info(msg)                     
                
                msg = {'FROM' : myDroneId,
                    'TO' : 'GroundStation',
                    'TYPE' : 'freeDrone'}
                publish_info(msg)''' 
                onMission = False
                
                break
            # In case a relay is denied, this drone has to 
            # inform the drone that it's following of that
            elif tmp['TYPE'] == 'relayDenied':                
                msg = {'FROM' : myDroneId,
                'TO' : droneToFollow,
                'TYPE' : 'relayDenied'}
                publish_info(msg) 
                #sleep(0.5)

                '''msg = {'FROM' : myDroneId,
                'TO' : 'GroundStation',
                'TYPE' : 'cancelRelay'}
                publish_info(msg) 

                #sleep(0.5)
                msg = {'FROM' : myDroneId,
                'TO' : 'GroundStation',
                'TYPE' : 'freeDrone'}
                publish_info(msg)'''
                onMission = False 
                break
        # Free thread
        sleep(0.01)

def main(args=None):
    global myDroneId
    global publisherCmd
    global publisherInfo
    global status_subscriber 
    global telem_subscriber
    global cmd_subscriber
    global info_subscriber
    global currentTelem
    global droneToFollow
    global onMission
    global lock

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', required=True)
    cli_args = parser.parse_args()

    myDroneId = cli_args.drone

    rclpy.init(args=args)

    node = rclpy.create_node('publisher' + myDroneId)
    publisherCmd = node.create_publisher(String, '/cmd', 10)
    publisherInfo = node.create_publisher(String, '/info', 10)

    # Init subscribers
    status_subscriber = StatusSubscriber()
    cmd_subscriber = CmdSubscriber()
    telem_subscriber = TelemSubscriber()
    info_subscriber = InfoSubscriber()

    # To make sure every message is sent
    sleep(5)    

    print("READY!")

    while True:

        # Wait until relay is assigned
        droneToFollow = waitForRelayRequest()
        cancel_cmd()

        onMission = True

        # Check if drone is on ground and if it is, takeoff
        while True:
            lock.acquire()
            rclpy.spin_once(telem_subscriber, timeout_sec=0.5)
            tmp = json.loads(currentTelem) 
            lock.release()
            if tmp['droneId'] == myDroneId:
                # Arm and takeoff drone in case it's landed
                if not tmp['armed']:
                    # Take off
                    takeoff_cmd()
                    waitForTask(cmd='takeoff',state='finish')
                break   
   
        # Mission handler
        relayHandler()

        #print("END WHILE TRUE LOOP")

        # Check if the drone is close enough to the GS to send freeDrone message
        while True:
            lock.acquire()
            rclpy.spin_once(telem_subscriber, timeout_sec=0.5)
            tmp = json.loads(currentTelem) 
            lock.release()
            if tmp['droneId'] == myDroneId:                
                Lat = tmp['position']['lat']
                Lon = tmp['position']['lon']
                if not Math.requiresRelay(GS_coords,(Lat,Lon),target_dist):
                    msg = {'FROM' : myDroneId,
                    'TO' : 'GroundStation',
                    'TYPE' : 'cancelRelay'}
                    publish_info(msg) 
   
                    msg = {'FROM' : myDroneId,
                    'TO' : 'GroundStation',
                    'TYPE' : 'freeDrone'}
                    publish_info(msg)
                    break   
    
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    
    # Destroy main node and shutdown
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()