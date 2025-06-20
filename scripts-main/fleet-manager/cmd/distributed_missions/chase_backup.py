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
myDroneId = ''
publisher = ''
droneToFollow = ''  
lock = threading.Lock()

# MQTT
serverURI = "localhost"
currentSensor = {"value": {"lat": -1, 
                  "lon": -1, 
                  "alt": -1}}

#Status subscribers
currentStatus = """{"test" : ""}"""
status_subscriber = None

#Telem subscribers
telem_subscriber = None
currentTelem = None
TelemOrigin = None
TelemTarget = None

def current_time_millis():
    return int(round(time.time() * 1000))

def base_cmd(droneId=None):
    global myDroneId
    if droneId == None:
        droneId = myDroneId

    return {'droneId' : droneId, 'mode' : 'action', 'timestamp' : current_time_millis()}

def publish_cmd(cmd):
    global publisher

    msg = String()    
    msg.data = json.dumps(cmd)
    publisher.publish(msg)

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

    currentSensor = json.loads(message.payload)
    
def main(args=None):
    global myDroneId
    global publisher
    global status_subscriber 
    global telem_subscriber
    global droneToFollow
    global serverURI
    global currentSensor

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', required=True)
    #parser.add_argument('-f', '--follow', required=True) 

    cli_args = parser.parse_args()

    myDroneId = cli_args.drone
    #droneToFollow = cli_args.follow

    rclpy.init(args=args)

    node = rclpy.create_node('publisher' + myDroneId)
    publisher = node.create_publisher(String, '/cmd', 10)

    status_subscriber = StatusSubscriber() 

    # MQTT
    client = mqtt.Client(myDroneId)
    client.connect(serverURI)
    client.on_message = callbackSensor
    client.subscribe("sensors")

    # To make sure every message is sent
    sleep(5)

    telem_subscriber = TelemSubscriber()

    takeoff_cmd()
    waitForTask(cmd='takeoff',state='finish') 

    isRunning = False

    '''while True:
        print("isRunning =",isRunning)
        targetLat, targetLon, targetAlt = chase(targetDrone=droneToFollow)
        if targetLat == None:
            if isRunning == True:
                cancel_cmd()
            isRunning = False
        else:          
            goto_cmd(targetLat, targetLon, alt=targetAlt)
            isRunning = True
        sleep(1)'''

    currentPosition = {"lat" : -1,
                       "lon" : -1,
                       "alt" : -1}

    while True:
        client.loop(0.5)
        # Check if the current coordinate is different from the previous
        if not (currentSensor['value']['lat'] == currentPosition['lat'] and currentSensor['value']['lon'] == currentPosition['lon'] and currentSensor['value']['alt'] == currentPosition['alt']):
            currentPosition['lat'] = currentSensor['value']['lat']
            currentPosition['lon'] = currentSensor['value']['lon']
            currentPosition['alt'] = currentSensor['value']['alt']
            goto_cmd(lat=currentSensor['value']['lat'],lon=currentSensor['value']['lon'],alt=currentSensor['value']['alt']+15,speed=10)
        
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    
    # Destroy main node and shutdown
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()