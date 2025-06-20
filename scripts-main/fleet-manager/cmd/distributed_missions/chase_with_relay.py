import argparse
import Math

from time import sleep
import time
import json
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading

# Global variables
currentPosition = None
client = None
myDroneId = ''
publisherCmd = ''
publisherInfo = ''
lock = threading.Lock()
onMission = False
droneFollowingMe = None
sensorToFollow = None
GS_coords = (40.63418785269594, -8.660018532516903)

# Distance to keep between nodes
target_dist = 65 # meters

# Error accepted in calculations
tolerance = 2 # meters

# MQTT
serverURI = "192.168.1.184"
currentSensor = None

# Info subscribers
currentInfo = """{"test" : ""}"""
info_subscriber = None

#Status subscribers
currentStatus = """{"test" : ""}"""
status_subscriber = None

#Telem subscribers
telem_subscriber = None
currentTelem = """{'droneId' : ""}"""
TelemOrigin = None
TelemTarget = """{'droneId' : ""}"""

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
       
def chase(originDrone=None,targetDrone=None):
    global telem_subscriber
    global currentTelem
    global myDroneId

    if originDrone == None:
        originDrone = myDroneId

    # Variables to save
    originLat = 0
    originLon = 0
    originAlt = 0
    targetLat = 0
    TargetLon = 0

    rclpy.spin_once(telem_subscriber)

    # Get origin position
    while True:
        tmp = json.loads(currentTelem)  
        if tmp['droneId'] == originDrone:
            originLat = tmp['position']['lat']
            originLon = tmp['position']['lon']
            originAlt = tmp['position']['alt']
            break
        rclpy.spin_once(telem_subscriber)

    # Get target position
    while True:
        tmp = json.loads(currentTelem)  
        if tmp['droneId'] == targetDrone:
            targetLat = tmp['position']['lat']
            TargetLon = tmp['position']['lon']            
            break
        rclpy.spin_once(telem_subscriber)

    return Math.chaseCalc(originLat,originLon,targetLat,TargetLon,originAlt,0.0001)   
   
def callbackSensor(client, userdata, message):
    global currentSensor
    print("callbackSensor()")
    currentSensor = json.loads(message.payload)

def checkIfRelayIsNeeded(Lat,Lon):
    global currentInfo
    global myDroneId
    global info_subscriber
    global GS_coords
    global target_dist
    global droneFollowingMe
    global onMission

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
                    return

    # If there's a drone following him
    else:
        # In case the relay needs to be cancelled
        if not Math.requiresRelay(GS_coords,(Lat,Lon),target_dist):
            # Send message to GS to inform he's not
            # being followed anymore
            msg = {'FROM' : myDroneId,
                'TO' : 'GroundStation',
                'TYPE' : 'cancelRelay'}
            publish_info(msg) 
            sleep(0.5) 
            # Send message to the drone that is following
            # that it can return home
            msg = {'FROM' : myDroneId,
            'TO' : droneFollowingMe,
            'TYPE' : 'freeDrone'}
            publish_info(msg) 
            droneFollowingMe = None    

def goto(Lat, Lon, alt=None, speed=None):
    global onMission

    if onMission:
        checkIfRelayIsNeeded(Lat,Lon)
        goto_cmd(Lat,Lon,alt=alt,speed=speed)

def chaseHandler(): 
    global onMission
    global client
    global currentPosition
    global currentSensor
    global sensorToFollow
    global droneFollowingMe
    global lock

    # Thread to check if relay has been cancelled
    checkChaseCancel = threading.Thread(target=checkForChaseCancel)
    checkChaseCancel.start()

    checkRelayCancel = threading.Thread(target=checkForRelayCancel)
    checkRelayCancel.start()

    while onMission:
        lock.acquire()
        client.loop(0.5)
        #print("chaseHandler()")    
        lock.release()    
        #print("currentSensor =",currentSensor)
        # Check if the current coordinate is different from the previous
        if currentSensor['type'] == "locator" and currentSensor['sensorId'] == sensorToFollow:
            if not (currentSensor['value']['lat'] == currentPosition['lat'] and currentSensor['value']['lon'] == currentPosition['lon'] and currentSensor['value']['alt'] == currentPosition['alt']):
                currentPosition['lat'] = currentSensor['value']['lat']
                currentPosition['lon'] = currentSensor['value']['lon']
                currentPosition['alt'] = currentSensor['value']['alt']
                goto(Lat=currentSensor['value']['lat'],Lon=currentSensor['value']['lon'],alt=currentSensor['value']['alt']+15,speed=10)
        # Free thread
        sleep(0.01)

    checkChaseCancel.join()
    checkRelayCancel.join()
    droneFollowingMe = None
    return_home_cmd()

def checkForChaseCancel():
    global myDroneId
    global info_subscriber
    global onMission
    global droneFollowingMe
    global sensorToFollow
    global currentSensor
    global lock
    global client

    while onMission:
        lock.acquire()
        print("checkForChaseCancel()")   
        client.loop(0.5)
        lock.release() 
        if currentSensor['type'] == 'locatorCancel' and currentSensor['sensorId'] == sensorToFollow:
            onMission = False
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
                'TYPE' : 'freeDroneChase'}
            publish_info(msg)'''
                 
        # Free thread
        sleep(0.01)         

def checkForRelayCancel():
    global myDroneId
    global info_subscriber
    global droneFollowingMe
    global onMission
    global lock
    global currentInfo
    global sensorToFollow
    global client

    while onMission:
        lock.acquire()   
        print("checkForRelayCancel()") 
        rclpy.spin_once(info_subscriber, timeout_sec=0.5)  
        tmp = json.loads(currentInfo)
        #print("currentInfo =",currentInfo)
        lock.release() 
        if tmp['TO'] == myDroneId:             
            # In case a relay is denied, this drone has to 
            # inform the the GS that is not being followed anymore
            if tmp['TYPE'] == 'relayDenied':               
                '''msg = {'FROM' : myDroneId,
                'TO' : 'GroundStation',
                'TYPE' : 'cancelRelay'}
                publish_info(msg)'''
                onMission = False
                droneFollowingMe = None  
                '''msg = {'FROM' : myDroneId,
                    'TO' : 'GroundStation',
                    'TYPE' : 'freeDroneChase'}
                publish_info(msg)'''

                message = {'FROM' : myDroneId,
                'TO' : sensorToFollow,
                'type' : 'chaseCanceled'}
                client.publish("sensors",json.dumps(message))
                break

        # Free thread
        sleep(0.01)

def waitForChaseRequest():
    global currentInfo
    global info_subscriber

    while True:
        print("waitForChaseRequest()")
        rclpy.spin_once(info_subscriber)
        tmp = json.loads(currentInfo) 
        #print("message = ",tmp)
        if tmp['FROM'] == 'GroundStation' and tmp['TO'] == myDroneId and tmp['TYPE'] == 'chaseAssigned':
            print("sensorAssigned")
            return tmp['sensorAssigned']
    
def main(args=None):
    global myDroneId
    global publisherCmd
    global publisherInfo
    global status_subscriber 
    global telem_subscriber
    global info_subscriber
    global serverURI
    global currentSensor
    global currentPosition
    global sensorToFollow
    global onMission
    global client
    global lock

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', required=True)
    #parser.add_argument('-f', '--follow', required=True) 

    cli_args = parser.parse_args()

    myDroneId = cli_args.drone
    #droneToFollow = cli_args.follow

    rclpy.init(args=args)

    node = rclpy.create_node('publisher' + myDroneId)
    publisherCmd = node.create_publisher(String, '/cmd', 10)
    publisherInfo = node.create_publisher(String, '/info', 10)

    status_subscriber = StatusSubscriber() 
    info_subscriber = InfoSubscriber()
    telem_subscriber = TelemSubscriber()

    # MQTT
    

    # To make sure every message is sent
    sleep(5)

    print("READY!")
    #isRunning = False

    while True:
        
        # Default Values
        currentPosition = {"lat" : -1,
                           "lon" : -1,
                           "alt" : -1}
        currentSensor = {"sensorId" : -1, 
                     "type" : -1,
                     "value": {"lat": -1, 
                     "lon": -1, 
                     "alt": -1}}

        # Wait until chase is assigned
        sensorToFollow = waitForChaseRequest()
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
               
        # MQTT
        client = mqtt.Client(myDroneId)
        client.connect(serverURI)
        client.on_message = callbackSensor
        client.subscribe("sensors")

        # Mission handler        
        chaseHandler()

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
                        'TYPE' : 'freeDroneChase'}
                    publish_info(msg)
                    break   

        # Disconnect from MQTT
        client.disconnect()

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    
    # Destroy main node and shutdown
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()