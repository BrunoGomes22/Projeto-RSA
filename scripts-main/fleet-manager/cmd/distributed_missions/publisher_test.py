import argparse

from time import sleep
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading

# Global variables
myDroneId = ''
publisher = ''

#Status subscribers
currentStatus = None
status_subscriber = None

#Telem subscribers
currentTelem = None

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
    print("PUBLISHING = ",msg.data)

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
        print("LISTENER")  

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
        print("LISTENER")        
        
def waitForTask(cmd=None,state=None,droneId=None):    
    global currentStatus
    global myDroneId
    global status_subscriber
    
    # Check first time         
    rclpy.spin_once(status_subscriber)
    tmp = ''    

    if droneId == None:
        droneId = myDroneId

    while True:        
        tmp = json.loads(currentStatus)  
        if 'command' in tmp:      
            if tmp['droneId'] == droneId and tmp['command'] == cmd and tmp['state'] == state :           
                break        
        rclpy.spin_once(status_subscriber)  
       
    
def main(args=None):
    global myDroneId
    global publisher
    global status_subscriber 

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', required=True)
    cli_args = parser.parse_args()

    myDroneId = cli_args.drone

    rclpy.init(args=args)

    node = rclpy.create_node('publisher' + myDroneId)
    publisher = node.create_publisher(String, '/cmd', 10)

    status_subscriber = StatusSubscriber() 

    # To make sure every message is sent
    sleep(5)

    takeoff_cmd()
    waitForTask(cmd='takeoff',state='finish') 

    goto_cmd(40.63444498571699, -8.660219161521956, alt=10)
    waitForTask(cmd='goto',state='finish')
    #sleep(5)

    goto_cmd(40.63570355154102, -8.662709273198388, alt=20)
    waitForTask(cmd='goto',state='finish')
    #sleep(5)

    goto_cmd(40.63336594059007, -8.664783033302115, alt=20)
    waitForTask(cmd='goto',state='finish')
    #sleep(5)

    goto_cmd(40.633346108530446, -8.662426610879557, alt=25)
    waitForTask(cmd='goto',state='finish')
    #sleep(5)

    goto_cmd(40.63342695487569, -8.660659109313242, alt=25)
    waitForTask(cmd='goto',state='finish')
    #sleep(5)

    land_cmd()
    waitForTask(cmd='land',state='finish') 
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    
    # Destroy main node and shutdown
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()